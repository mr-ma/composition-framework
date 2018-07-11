#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/graph/filtered_graph.hpp>
#include <composition/graph/graph.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/scc.hpp>
#include <composition/graph/topological_sort.hpp>

namespace composition {
typedef unsigned long ProtectionIndex;
class ProtectionGraph {
private:
  graph_t Graph{};
  ProtectionIndex ProtectionIdx{};
  std::unordered_map<ProtectionIndex, ManifestIndex> Protections{};

private:
  vd_t insertNode(llvm::Value *node, vertex_type type);

  void expandBasicBlockToInstructions(vd_t B, llvm::BasicBlock *pBlock);

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
                                                           Graph(that.Graph) {
  };

  ProtectionGraph &operator=(ProtectionGraph &&) = default;

  graph_t &getGraph();

  ProtectionIndex addConstraint(ManifestIndex index, std::shared_ptr<Constraint> c);

  template<typename T, typename S>
  uintptr_t addCFG(T parent, S child) {
    assert(parent != nullptr);
    assert(child != nullptr);
    auto srcNode = this->insertNode(parent, llvmToVertexType(parent));
    auto dstNode = this->insertNode(child, llvmToVertexType(child));

    auto edge = boost::add_edge(srcNode, dstNode, Graph);
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
      assert(G != nullptr);
      return (*G)[ed].type == edge_type::DEPENDENCY;
    }

    bool operator()(typename T::vertex_descriptor vd) const {
      assert(G != nullptr);
      return true;
    }

    T *G;

    Predicate() = default;

    explicit Predicate(T *G) : G(G) {}
  };

  void SCC_DEPENDENCY(graph_t &g) {
    Predicate<graph_t> p(&g);
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

    std::vector<uintptr_t> leftProtections;
    graph_t::edge_iterator it, it_end;
    for (std::tie(it, it_end) = boost::edges(g); it != it_end; ++it) {
      auto e = g[*it];
      leftProtections.push_back(e.index);
    }
    for (uintptr_t i = 0; i < ProtectionIdx; i++) {
      if (std::find(leftProtections.begin(), leftProtections.end(), i) == leftProtections.end()) {
        removeProtection(i);
        composition::ManifestRegistry::Remove(i);
      }
    }
  }

  void handleCycle(std::vector<vd_t> matches);

  std::vector<vd_t> topologicalSortProtections() {
    Predicate<graph_t> p(&Graph);
    auto fg = boost::make_filtered_graph(Graph, p);
    return composition::reverse_topological_sort(fg);
  }
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
