#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#include <utility>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/copy.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/graph.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/graph/algorithm/strong_components.hpp>
#include <composition/graph/algorithm/topological_sort.hpp>
#include <composition/graph/util/index_map.hpp>
#include <composition/graph/util/vertex_count.hpp>

namespace composition {
using ProtectionIndex = unsigned long;
using VertexIndex = uintptr_t;
using EdgeIndex = uintptr_t;

class ProtectionGraph {
private:
  graph_t Graph{};
  ProtectionIndex ProtectionIdx{};
  std::unordered_map<ProtectionIndex, ManifestIndex> Protections{};
  std::unordered_map<VertexIndex, vd_t> vertices{};
  std::unordered_map<EdgeIndex, ed_t> edges{};

private:
  vd_t add_vertex(llvm::Value *v);

  void remove_vertex(vd_t vd) noexcept;

  ed_t add_edge(vd_t s, vd_t d, edge_t e);

  void remove_edge(ed_t ed) noexcept;

  void expandBasicBlockToInstructions(vd_t it, llvm::BasicBlock *B);

  void expandInstructionToFunction(vd_t it, llvm::Instruction *I);

  void expandBasicBlockToFunction(vd_t it, llvm::BasicBlock *B);

  void replaceTarget(vd_t src, vd_t dst);

  void replaceTargetIncomingEdges(vd_t src, vd_t dst);

  void replaceTargetOutgoingEdges(vd_t src, vd_t dst);

public:
  ProtectionGraph() = default;

  ProtectionGraph(const ProtectionGraph &that) = delete;

  ProtectionGraph(const ProtectionGraph &&that) noexcept : ProtectionIdx(that.ProtectionIdx),
                                                           Protections(that.Protections),
                                                           Graph(that.Graph),
                                                           vertices(that.vertices),
                                                           edges(that.edges) {
  }

  ProtectionGraph &operator=(ProtectionGraph &&that) noexcept {
    ProtectionIdx = that.ProtectionIdx;
    Protections = std::move(that.Protections);

    auto index = index_map(that.Graph);
    boost::copy_graph(that.Graph, this->Graph, vertex_index_map(boost::make_assoc_property_map(index)));
    return *this;
  };

  graph_t &getGraph();

  ProtectionIndex addConstraint(ManifestIndex index, std::shared_ptr<Constraint> c);

  template<typename T, typename S>
  ProtectionIndex addCFG(T parent, S child) {
    assert(parent != nullptr);
    assert(child != nullptr);
    auto srcNode = this->add_vertex(parent);
    auto dstNode = this->add_vertex(child);
    this->add_edge(srcNode, dstNode, edge_t{ProtectionIdx, "CFG", edge_type::CFG});
    return ProtectionIdx++;
  }

  std::vector<ManifestIndex> manifestIndexes(bool requireTopologicalSort = false);

  void removeProtection(ProtectionIndex protectionID);

  void expandToInstructions();

  void reduceToInstructions();

  void expandToFunctions();

  void reduceToFunctions();

  template<typename T>
  void dependencyConflictHandling(T &g) {
    auto rg = filter_removed_graph(g);
    auto fg = filter_dependency_graph(rg);

    bool changed;
    do {
      changed = false;
      auto components = strong_components(fg);
      int i = 0;
      for (auto &component : components) {
        auto vertexCount = vertex_count(component);
        if (vertexCount != 1) {
          changed = true;
          llvm::dbgs() << "Component " << std::to_string(i) << " contains cycle with " << std::to_string(vertexCount)
                       << " elements.\n";
          save_graph_to_dot(component, "graph_component_" + std::to_string(i) + ".dot");
          save_graph_to_graphml(component, "graph_component_" + std::to_string(i) + ".graphml");
          handleCycle(component);
          ++i;
        }
      }
    } while (changed);

    std::vector<ProtectionIndex> leftProtections;
    for (auto[it, it_end] = boost::edges(g); it != it_end; ++it) {
      auto e = g[*it];
      leftProtections.push_back(e.index);
    }

    for (ProtectionIndex i = 0; i < ProtectionIdx; i++) {
      if (std::find(leftProtections.begin(), leftProtections.end(), i) != leftProtections.end()) {
        continue;
      }

      //Filtering for Dependency edge is needed as we otherwise remove manifests which contain other constraints
      //like preserved or present as of right now
      if (edges.find(i) == edges.end()) {
        continue;
      }

      if (g[edges[i]].type != edge_type::DEPENDENCY) {
        continue;
      }

      ManifestRegistry::Remove(Protections[i]);
      removeProtection(i);
    }
  }

  template<typename graph_t>
  void handleCycle(graph_t &g) {
    llvm::dbgs() << "Handling cycle in component\n";

    vd_t prev = nullptr;
    for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
      vd_t vd = *vi;
      auto v = g[vd];

      llvm::dbgs() << v.name << "\n";
      llvm::dbgs() << std::to_string(v.index) << "\n";

      if (prev == nullptr) {
        prev = vd;
        continue;
      }

      for (auto edge = boost::edge(vd, prev, g); edge.second; edge = boost::edge(vd, prev, g)) {
        remove_edge(edge.first);
      }
      prev = vd;
    }
  }

  std::vector<vd_t> topologicalSortProtections() {
    graph_t &g = Graph;
    auto fg = filter_dependency_graph(g);
    return topological_sort(fg);
  }
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
