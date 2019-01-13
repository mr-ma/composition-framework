#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP

#include <algorithm>
#include <array>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/constraint/constraint.hpp>
#include <composition/graph/constraint/true.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Performance.hpp>
#include <composition/profiler.hpp>
#include <composition/support/options.hpp>
#include <cstdint>
#include <glpk.h>
#include <lemon/graph_to_eps.h>
#include <lemon/list_graph.h>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <random>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace composition::graph {
using composition::graph::constraint::Constraint;
using composition::graph::constraint::constraint_idx_t;
using composition::graph::constraint::True;
using composition::graph::util::graph_to_dot;
using composition::graph::util::graph_to_graphml;
using composition::metric::Performance;
using composition::support::cStats;

using ManifestUndoMap =
    boost::bimaps::bimap<boost::bimaps::multiset_of<manifest_idx_t>, boost::bimaps::multiset_of<llvm::Value*>>;
using ManifestProtectionMap =
    boost::bimaps::bimap<boost::bimaps::multiset_of<manifest_idx_t>, boost::bimaps::multiset_of<manifest_idx_t>>;
using ManifestDependencyMap =
    boost::bimaps::bimap<boost::bimaps::multiset_of<manifest_idx_t>, boost::bimaps::multiset_of<manifest_idx_t>>;

/**
 * The core of the composition framework, the protection graph and its algorithms
 */
class ProtectionGraph {
private:
  /**
   * The graph in boost representation
   */
  lemon::ListDigraph LG{};
  std::unique_ptr<lemon::ListDigraph::NodeMap<vertex_t>> vertices;
  std::unique_ptr<lemon::ListDigraph::ArcMap<edge_t>> edges;
  std::unordered_map<vertex_idx_t, lemon::ListDigraph::Node> VERTICES_DESCRIPTORS{};
  std::unordered_map<edge_idx_t, lemon::ListDigraph::Arc> EDGES_DESCRIPTORS{};

  /**
   * The current strictly increasing vertex index
   */
  vertex_idx_t VertexIdx{};
  /**
   * The current strictly increasing edge index
   */
  edge_idx_t EdgeIdx{};
  /**
   * Cache of vertices for faster lookup
   */
  using vertex_cache_t = std::unordered_map<llvm::Value*, vertex_idx_t>;
  vertex_cache_t vertexRealCache{};
  vertex_cache_t vertexShadowCache{};
  std::array<vertex_cache_t*, 2> vertexCache = {{&vertexRealCache, &vertexShadowCache}};

  /**
   * Map which captures the undo relationship between manifests.
   */
  ManifestDependencyMap DependencyUndo{};
  /**
   * Map which captures the protection relationship between manifests.
   */
  ManifestProtectionMap ManifestProtection{};

  std::unordered_map<manifest_idx_t, Manifest*> MANIFESTS;

  constraint_idx_t ConstraintIdx{};

  boost::bimaps::bimap<boost::bimaps::multiset_of<manifest_idx_t>, constraint_idx_t> MANIFESTS_CONSTRAINTS{};
  std::unordered_map<constraint_idx_t, vertex_idx_t> CONSTRAINTS_VERTICES{};
  std::unordered_map<constraint_idx_t, edge_idx_t> CONSTRAINTS_EDGES{};

private:
  /**
   * Adds a vertex with value `v` to the graph if it does not exist.
   * @param v the llvm value
   * @return a vertex descriptor for the added/existing vertex
   */
  vertex_idx_t add_vertex(llvm::Value* value, bool shadow);
  vertex_idx_t add_vertex(
      llvm::Value* value,
      const std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>& constraints);

  /**
   * Adds an edge to the graph.
   * @param s the source vertex
   * @param d the destination vertex
   * @param e the associated edge information
   * @return an edge descriptor pointing to the added edge
   */
  edge_idx_t add_edge(vertex_idx_t s, vertex_idx_t d);
  edge_idx_t add_edge(
      vertex_idx_t s, vertex_idx_t d,
      const std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>& constraints);

public:
  ProtectionGraph();

  /**
   * Destroys the graph and frees memory
   */
  void destroy();

  size_t countVertices();
  size_t countEdges();

  const ManifestDependencyMap getManifestDependencyMap() const { return DependencyUndo; }

  const ManifestProtectionMap getManifestProtectionMap() const { return ManifestProtection; }

  void addManifests(std::set<Manifest*> manifests);
  void addManifest(Manifest* m);

  void Print(std::string name) {
    auto texts = lemon::ListDigraph::NodeMap<std::string>(LG);
    auto coords = lemon::ListDigraph::NodeMap<lemon::dim2::Point<int>>(LG);
    auto sizes = lemon::ListDigraph::NodeMap<int>(LG);
    int i = 0;
    for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
      texts[n] = (*vertices)[n].name;
      coords[n] = lemon::dim2::Point<int>(i % 2, i / 2);
      sizes[n] = 1;
      ++i;
    }

    auto widths = lemon::ListDigraph::ArcMap<double>(LG);
    for (lemon::ListDigraph::ArcIt e(LG); e != lemon::INVALID; ++e) {
      widths[e] = 0.5;
    }

    lemon::graphToEps(LG, name + ".eps")
        .nodeSizes(sizes)
        .negateY()
        //.absoluteNodeSizes()
        .coords(coords)
        .nodeTexts(texts)
        .nodeTextSize(0.05)
        .drawArrows()
        .arrowWidth(0.1)
        .edgeWidths(widths)
        .absoluteEdgeWidths()
        .run();
  }

  /**
   * Adds a constraint to the protection graph
   * @param m the manifest associated with the constraint
   * @param c the constraint
   * @return a unique protection index
   */
  constraint_idx_t addConstraint(manifest_idx_t idx, std::shared_ptr<Constraint> c);

  void addHierarchy(llvm::Module& M);

  void connectShadowNodes();

  /**
   * Performs a topological sorting of the given manifests according to the protection graph
   * @param manifests to sort
   * @return the sorted manifests
   */
  std::vector<Manifest*> topologicalSortManifests(std::set<Manifest*> manifests);

  /**
   * Computes the dependency relationship of the manifests.
   */
  void computeManifestDependencies();

  std::set<std::pair<manifest_idx_t, manifest_idx_t>> vertexConflicts();
  std::set<std::pair<manifest_idx_t, manifest_idx_t>> computeDependencies();
  std::set<std::set<manifest_idx_t>> computeCycles();
  /**
   * Detects and handles the conflicts in the graph `g`
   * @tparam graph_t the type of the graph `g`
   * @param g the graph
   * @param strategy the strategy to use for handling conflicts
   */
  std::set<Manifest*> conflictHandling(llvm::Module& M);

  template <typename graph_t>
  void prepareHotness(graph_t& g, const std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*>& BFI) {
    /*llvm::dbgs() << "Calculating hotness\n";
    auto rg = g;

    uint64_t maxHotness = 0;
    for (auto [vi, vi_end] = boost::vertices(rg); vi != vi_end; ++vi) {
      if (g[*vi].type != vertex_type::INSTRUCTION) {
        continue;
      }
      auto* value = static_cast<llvm::Instruction*>(g[*vi].value);
      if (value == nullptr || value->getParent() == nullptr || value->getParent()->getParent() == nullptr) {
        continue;
      }

      if (BFI.find(value->getParent()->getParent()) == BFI.end()) {
        continue;
      }

      auto hotness = Performance::getBlockFreq(value->getParent(), BFI.at(value->getParent()->getParent()), false);
      g[*vi].absoluteHotness = hotness;

      if (hotness > maxHotness) {
        maxHotness = hotness;
      }
    }

    for (auto [vi, vi_end] = boost::vertices(rg); vi != vi_end; ++vi) {
      if (g[*vi].type != vertex_type::INSTRUCTION) {
        continue;
      }
      g[*vi].hotness = g[*vi].absoluteHotness / static_cast<float>(maxHotness);
    }*/
  }

  template <typename graph_t>
  void removeHotNodes(graph_t& g, double coverage, llvm::Module* module, std::vector<Manifest*> manifests,
                      std::unordered_set<llvm::Instruction*> allInstructions) {
    /*llvm::dbgs() << "Removing hot nodes\n";

    while (true) {
      llvm::dbgs() << "Loop\n";
      std::unordered_map<std::string, std::unordered_set<llvm::Instruction*>> instructionProtections{};

      for (auto& m : manifests) {
        for (auto& I : m->Coverage()) {
          instructionProtections[m->name].insert(I);
        }
      }

      std::unordered_map<llvm::Instruction*, size_t> instructionConnectivityMap{};
      for (auto& I : allInstructions) {
        instructionConnectivityMap[I] = 0;
      }
      for (auto& [p, instr] : instructionProtections) {
        for (auto* I : instr) {
          instructionConnectivityMap[I]++;
        }
      }

      std::vector<size_t> connectivity{};
      for (auto& [I, c] : instructionConnectivityMap) {
        connectivity.push_back(c);
      }
      composition::metric::Connectivity instConnectivity{connectivity};

      if (instConnectivity.avg <= coverage || manifests.empty()) {
        break;
      }

      std::vector<typename graph_t::vertex_descriptor> vertices{};
      float maxHotness = 0;
      auto rg = g;
      for (auto [vi, vi_end] = boost::vertices(rg); vi != vi_end; ++vi) {
        if (g[*vi].hotness > maxHotness) {
          maxHotness = g[*vi].hotness;
          vertices.clear();
          vertices.push_back(*vi);
        } else if (g[*vi].hotness == maxHotness) {
          vertices.push_back(*vi);
        }
      }

      std::unordered_set<Manifest*> candidates{};
      for (auto vd : vertices) {
        for (auto [cIdx, c] : g[vd].constraints) {
          manifest_idx_t idx = MANIFESTS_CONSTRAINTS.right.find(cIdx)->second;
          Manifest* m = MANIFESTS.find(idx)->second;
          candidates.insert(m);
        }
      }

      int minCoverage = 0;
      Manifest* best = nullptr;

      for (auto m : candidates) {
        auto localCoverage = m->Coverage().size();
        if (best == nullptr || localCoverage > minCoverage) {
          minCoverage = localCoverage;
          best = m;
        }
      }
      if (best == nullptr) {
        break;
      }

      llvm::dbgs() << "Remove\n";
      removeManifest(best->index);

      manifests.clear();
      for (auto mi = Protections.left.begin(), mi_end = Protections.left.end(); mi != mi_end; ++mi) {
        manifests.push_back(MANIFESTS.find(mi->first)->second);
      }
    }*/
  }
};
} // namespace composition::graph
#endif // COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
