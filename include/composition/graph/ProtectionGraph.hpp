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
#include <composition/graph/filtered_graph_hidden.hpp>

namespace composition {
typedef unsigned long ProtectionIndex;
class ProtectionGraph {
private:
  graph_t GraphWithHidden{};
  graph_hidden_t Graph;
  ProtectionIndex ProtectionIdx{};
  std::unordered_map<ProtectionIndex, ManifestIndex> Protections{};
  std::unordered_map<uintptr_t, vd_t> vertices;

private:
  vd_t insertNode(llvm::Value *node, vertex_type type);

  void expandBasicBlockToInstructions(vd_t B, llvm::BasicBlock *pBlock);

  void expandInstructionToFunction(vd_t it, llvm::Instruction *I);

  void expandBasicBlockToFunction(vd_t it, llvm::BasicBlock *B);

  void replaceTarget(vd_t src, vd_t dst);

  void replaceTargetIncomingEdges(vd_t src, vd_t dst);

  void replaceTargetOutgoingEdges(vd_t src, vd_t dst);

public:
  ProtectionGraph() : Graph({GraphWithHidden, HiddenPredicate<graph_t>(GraphWithHidden)}) {
  }

  ProtectionGraph(const ProtectionGraph &that) = delete;

  ProtectionGraph(const ProtectionGraph &&that) noexcept : ProtectionIdx(that.ProtectionIdx),
                                                           Protections(that.Protections),
                                                           GraphWithHidden(that.GraphWithHidden),
                                                           Graph(that.Graph) {
  }

  ProtectionGraph &operator=(ProtectionGraph &&that) noexcept {
    ProtectionIdx = that.ProtectionIdx;
    Protections = std::move(that.Protections);

    std::map<vd_t, size_t> index;
    auto pmap = boost::make_assoc_property_map(index);

    const auto &it = boost::make_iterator_range(boost::vertices(that.Graph));
    int idx = 0;
    for (auto vd : it) {
      put(pmap, vd, idx++);
    }

    boost::copy_graph(that.GraphWithHidden, this->GraphWithHidden, vertex_index_map(pmap));
    return *this;
  };

  graph_t &getGraphWithHidden();
  graph_hidden_t &getGraph();

  ProtectionIndex addConstraint(ManifestIndex index, std::shared_ptr<Constraint> c);

  template<typename T, typename S>
  ProtectionIndex addCFG(T parent, S child) {
    assert(parent != nullptr);
    assert(child != nullptr);
    auto srcNode = this->insertNode(parent, llvmToVertexType(parent));
    auto dstNode = this->insertNode(child, llvmToVertexType(child));

    auto edge = boost::add_edge(srcNode, dstNode, GraphWithHidden);
    assert(edge.second);
    Graph[edge.first] = edge_t{ProtectionIdx, "CFG", edge_type::CFG};
    //Protections[ProtectionIdx] = Protection(parent, child);
    return ProtectionIdx++;
  }

  void removeProtection(ProtectionIndex protectionID);

  void expandToInstructions();

  void reduceToInstructions();

  void expandToFunctions();

  void reduceToFunctions();

  template<typename T>
  struct Predicate { // both edge and vertex
    bool operator()(typename T::edge_descriptor ed) const {
      assert(g != nullptr);
      return (*g)[ed].type == edge_type::DEPENDENCY;
    }

    bool operator()(typename T::vertex_descriptor vd) const {
      assert(g != nullptr);
      return true;
    }

    T *g;

    Predicate() : g(nullptr) {}

    explicit Predicate(T &g) : g(&g) {}
  };

  template<typename T>
  void SCC_DEPENDENCY(T &g) {
    Predicate<T> p(g);
    auto fg = boost::make_filtered_graph(g, p);

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
      if (std::find(leftProtections.begin(), leftProtections.end(), i) == leftProtections.end()) {
        removeProtection(i);
        composition::ManifestRegistry::Remove(i);
      }
    }
  }

  void handleCycle(std::vector<vd_t> matches);

  std::vector<vd_t> topologicalSortProtections() {
    Predicate<graph_hidden_t> p(Graph);
    auto fg = boost::make_filtered_graph(Graph, p);
    return composition::reverse_topological_sort(fg);
  }
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
