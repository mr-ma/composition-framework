#include <utility>

#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/copy.hpp>
#include <composition/graph/graph.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/scc.hpp>
#include <composition/graph/topological_sort.hpp>

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

  void removeProtection(ProtectionIndex protectionID);

  void expandToInstructions();

  void reduceToInstructions();

  void expandToFunctions();

  void reduceToFunctions();

  template<typename T>
  void SCC_DEPENDENCY(T &g) {
    auto fg = filter_dependency_graph(g);

    bool changed;
    do {
      changed = false;
      auto components = SCC(fg);
      int i = 0;
      for (const auto &matches : components) {
        if (matches.size() != 1) {
          changed = true;
          llvm::dbgs() << "Component " << std::to_string(i++) << " contains cycle with "
                       << std::to_string(matches.size()) << " elements.\n";
          handleCycle(matches);
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

      removeProtection(i);
      composition::ManifestRegistry::Remove(i);
    }
  }

  void handleCycle(std::vector<vd_t> matches);

  std::vector<vd_t> topologicalSortProtections() {
    graph_t &g = Graph;
    auto fg = filter_dependency_graph(g);
    return composition::reverse_topological_sort(fg);
  }
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
