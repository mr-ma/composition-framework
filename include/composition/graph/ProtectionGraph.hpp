#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#include <algorithm>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/algorithm/strong_components.hpp>
#include <composition/graph/algorithm/topological_sort.hpp>
#include <composition/graph/constraint/constraint.hpp>
#include <composition/graph/filter/dependency.hpp>
#include <composition/graph/filter/removed.hpp>
#include <composition/graph/filter/selfcycle.hpp>
#include <composition/graph/graph.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/graph/util/index_map.hpp>
#include <composition/graph/util/vertex_count.hpp>
#include <composition/metric/Performance.hpp>
#include <composition/profiler.hpp>
#include <composition/strategy/Random.hpp>
#include <composition/strategy/Strategy.hpp>
#include <composition/support/options.hpp>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <random>
#include <utility>

namespace composition::graph {
using graph::algorithm::strong_components;
using graph::algorithm::topological_sort;
using graph::constraint::Constraint;
using graph::constraint::Present;
using graph::constraint::PresentConstraint;
using graph::constraint::Preserved;
using graph::constraint::PreservedConstraint;
using graph::util::graph_to_dot;
using graph::util::graph_to_graphml;
using metric::Performance;
using support::cStats;
using util::constraint_map;
using util::index_map;
using util::vertex_count;

// ProtectionIndex type. TODO C++ does not enforce type safety. Potentially there are ways how type safety can be
// improved.
using ProtectionIndex = unsigned long;
// VertexIndex type. TODO C++ does not enforce type safety. Potentially there are ways how type safety can be improved.
using VertexIndex = uintptr_t;
// EdgeIndex type. TODO C++ does not enforce type safety. Potentially there are ways how type safety can be improved.
using EdgeIndex = uintptr_t;

/*
 * TODO: The framework makes use of bidirectional maps. However, the performance impact is higher than wanted.
 * A different structure, or a custom bidirectional map might result in better performance.
 */
// Edge cache for faster lookups
using EdgeCacheMap = boost::bimaps::bimap<boost::bimaps::multiset_of<EdgeIndex>, ed_t>;

// ProtectionMap for faster lookups
using ProtectionMap = boost::bimaps::bimap<boost::bimaps::multiset_of<Manifest*>, ProtectionIndex>;

using ManifestUndoMap =
    boost::bimaps::bimap<boost::bimaps::multiset_of<Manifest*>, boost::bimaps::multiset_of<llvm::Value*>>;
using ManifestProtectionMap =
    boost::bimaps::bimap<boost::bimaps::multiset_of<Manifest*>, boost::bimaps::multiset_of<Manifest*>>;
using ManifestDependencyMap =
    boost::bimaps::bimap<boost::bimaps::multiset_of<Manifest*>, boost::bimaps::multiset_of<Manifest*>>;

/**
 * The core of the composition framework, the protection graph and its algorithms
 */
class ProtectionGraph {
private:
  /**
   * The graph in boost representation
   */
  graph_t Graph{};
  /**
   * The current strictly increasing protection index
   */
  ProtectionIndex ProtectionIdx{};
  /**
   * A map of all protections
   */
  ProtectionMap Protections{};
  /**
   * The current strictly increasing vertex index
   */
  VertexIndex VertexIdx{};
  /**
   * Cache of vertices for faster lookup
   */
  std::unordered_map<llvm::Value*, vd_t> vertexCache{};
  /**
   * Cache of edges for faster lookup
   */
  EdgeCacheMap edgeCache{};
  /**
   * Map which captures the undo relationship between manifests.
   */
  ManifestDependencyMap DependencyUndo{};
  /**
   * Map which captures the protection relationship between manifests.
   */
  ManifestProtectionMap ManifestProtection{};

private:
  /**
   * Adds a vertex with value `v` to the graph if it does not exist.
   * @param v the llvm value
   * @return a vertex descriptor for the added/existing vertex
   */
  vd_t add_vertex(llvm::Value* v);

  /**
   * Removes a vertex from the graph
   * @param vd the vertex descriptor
   */
  void remove_vertex(vd_t vd) noexcept;

  /**
   * Adds an edge to the graph.
   * @param s the source vertex
   * @param d the destination vertex
   * @param e the associated edge information
   * @return an edge descriptor pointing to the added edge
   */
  ed_t add_edge(vd_t s, vd_t d, edge_t e);

  /**
   * Removes an edge from the graph
   * @param ed the edge descriptor
   */
  void remove_edge(ed_t ed) noexcept;

  /**
   * Remaps all constraints/edges of basicblocks to instructions
   * @param it the vertex of the basicblock
   * @param B the llvm basicblock
   */
  void expandBasicBlockToInstructions(vd_t it, llvm::BasicBlock* B);

  /**
   * Replaces the source with the destination vertex
   * @param src the source vertex descriptor
   * @param dst the destination vertex descriptor
   */
  void replaceTarget(vd_t src, vd_t dst);

  /**
   * Replaces the in-edges of the source with the destination vertex
   * @param src the source vertex descriptor
   * @param dst the destination vertex descriptor
   */
  void replaceTargetInEdges(vd_t src, vd_t dst);

  /**
   * Replaces the out-edges of the source with the destination vertex
   * @param src the source vertex descriptor
   * @param dst the destination vertex descriptor
   */
  void replaceTargetOutEdges(vd_t src, vd_t dst);

public:
  ProtectionGraph() = default;

  /**
   * Destroys the graph and frees memory
   */
  void destroy();

  graph_t& getGraph();

  const ManifestDependencyMap getManifestDependencyMap() const { return DependencyUndo; }

  const ManifestProtectionMap getManifestProtectionMap() const { return ManifestProtection; }

  /**
   * Adds a constraint to the protection graph
   * @param m the manifest associated with the constraint
   * @param c the constraint
   * @return a unique protection index
   */
  ProtectionIndex addConstraint(Manifest* m, std::shared_ptr<Constraint> c);

  /**
   * Adds CFG edges to the graph
   * @param parent the source llvm value
   * @param child the target llvm value
   * @return a unique protection index
   */
  ProtectionIndex addCFG(llvm::Value* parent, llvm::Value* child) {
    assert(parent != nullptr);
    assert(child != nullptr);
    auto srcNode = this->add_vertex(parent);
    auto dstNode = this->add_vertex(child);
    this->add_edge(srcNode, dstNode, edge_t{ProtectionIdx, "CFG", edge_type::CFG});
    return ProtectionIdx++;
  }

  /**
   * Performs a topological sorting of the given manifests according to the protection graph
   * @param manifests to sort
   * @return the sorted manifests
   */
  std::vector<Manifest*> topologicalSortManifests(std::unordered_set<Manifest*> manifests);

  /**
   * Removes a manifest from the graph
   * @param m the manifest to remove
   */
  void removeManifest(Manifest* m);

  /**
   * Expands all larger llvm values (module, function, basicblock) to an equivalent instruction representation
   */
  void expandToInstructions();

  /**
   * Removes all vertices which are not instructions or llvm values.
   */
  void reduceToInstructions();

  /**
   * Computes the dependency relationship of the manifests.
   */
  void computeManifestDependencies();

  /**
   * Checks if the graph `g` contains a cycle
   * @tparam graph_t the type of the graph `g`
   * @param g the graph
   * @return true if the graph contains at least one cycle, false otherwise
   */
  template <typename graph_t> bool hasCycle(graph_t& g) {
    Profiler sccProfiler{};
    try {
      topological_sort(g);
      cStats.timeConflictDetection += sccProfiler.stop();
      return false;
    } catch (boost::not_a_dag&) {
      cStats.timeConflictDetection += sccProfiler.stop();
      return true;
    }
    return false;
  }

  /**
   * Detects and handles the conflicts in the graph `g`
   * @tparam graph_t the type of the graph `g`
   * @param g the graph
   * @param strategy the strategy to use for handling conflicts
   */
  template <typename graph_t> void conflictHandling(graph_t& g, const std::unique_ptr<strategy::Strategy>& strategy) {
    llvm::dbgs() << "Step 1: Removing cycles...\n";
    auto rg = graph::filter::filter_removed_graph(g);
    auto fg = graph::filter::filter_dependency_graph(rg);
    auto sc = graph::filter::filter_selfcycle_graph(fg);

    // First detect cycles and remove all cycles.
    bool hadConflicts;
    Profiler sccProfiler{};
    Profiler resolvingProfiler{};
    do {
      hadConflicts = false;
      sccProfiler.reset();

      bool hasOneCycle = hasCycle(sc);
      if (!hasOneCycle) {
        cStats.timeConflictDetection += sccProfiler.stop();
        break;
      }

      auto components = strong_components(sc);
      cStats.timeConflictDetection += sccProfiler.stop();
      int i = 0;
      for (auto& component : components) {
        auto vertexCount = vertex_count(component);
        if (vertexCount != 1) {
          hadConflicts = true;
          llvm::dbgs() << "Component " << std::to_string(i) << " contains cycle with " << std::to_string(vertexCount)
                       << " elements.\n";
          if (support::DumpGraphs) {
            graph_to_dot(component, "graph_component_" + std::to_string(i) + ".dot");
            graph_to_graphml(component, "graph_component_" + std::to_string(i) + ".graphml");
          }
          do {
            resolvingProfiler.reset();
            handleCycle(component, strategy);
            cStats.timeConflictResolving += resolvingProfiler.stop();
          } while (hasCycle(component));
          ++i;
        }
      }
    } while (hadConflicts);

    llvm::dbgs() << "Step 2: Removing remaining present/preserved conflicts...\n";
    // Then remove remaining present/preserved conflicts.
    do {
      hadConflicts = false;

      sccProfiler.reset();
      auto [presentManifests, preservedManifests] = detectPresentPreservedConflicts(g);
      cStats.timeConflictDetection += sccProfiler.stop();
      if (!presentManifests.empty() || !preservedManifests.empty()) {
        hadConflicts = true;
        llvm::dbgs() << "Handling conflict...\n";
        cStats.conflicts++;

        resolvingProfiler.reset();
        std::vector<Manifest*> merged{};
        std::set_union(presentManifests.begin(), presentManifests.end(), preservedManifests.begin(),
                       preservedManifests.end(), std::back_inserter(merged));

        removeManifest(strategy->decidePresentPreserved(merged));
        cStats.timeConflictResolving += resolvingProfiler.stop();
      }
    } while (hadConflicts);
  }

  /**
   * Handles a cycle in the graph
   * @tparam graph_t the type of the graph `g`
   * @param g the graph
   * @param strategy the strategy to handle the cycle
   */
  template <typename graph_t> void handleCycle(graph_t& g, const std::unique_ptr<strategy::Strategy>& strategy) {
    llvm::dbgs() << "Handling cycle in component\n";
    cStats.cycles++;

    std::unordered_set<Manifest*> cyclicManifests{};
    for (auto [ei, ei_end] = boost::edges(g); ei != ei_end; ++ei) {
      cyclicManifests.insert(Protections.right.find(g[*ei].index)->second);
    }

    std::vector<Manifest*> merged{};
    for (auto& el : cyclicManifests) {
      merged.push_back(el);
    }

    removeManifest(strategy->decideCycle(merged));
  }

  /**
   * Detects present/preserved conflicts in the graph
   * @tparam graph_t the type of the graph `g`
   * @param g the graph
   * @return the detected present/preserved conflicting manifests
   */
  template <typename graph_t>
  std::pair<std::unordered_set<Manifest*>, std::unordered_set<Manifest*>> detectPresentPreservedConflicts(graph_t& g) {
    auto [isPresent, isPreserved] = constraint_map<PresentConstraint, PreservedConstraint>(g);

    std::unordered_set<Manifest*> presentManifests{};
    for (auto [vd, p] : isPresent) {
      if (p != PresentConstraint::CONFLICT) {
        continue;
      }

      for (auto [index, c] : g[vd].constraints) {
        if (llvm::isa<Present>(c.get())) {
          presentManifests.insert(Protections.right.find(index)->second);
        }
      }
    }

    std::unordered_set<Manifest*> preservedManifests{};
    for (auto [vd, p] : isPreserved) {
      if (p != PreservedConstraint::CONFLICT) {
        continue;
      }
      for (auto [index, c] : g[vd].constraints) {
        if (llvm::isa<Preserved>(c.get())) {
          preservedManifests.insert(Protections.right.find(index)->second);
        }
      }
    }

    return {presentManifests, preservedManifests};
  }

  template <typename graph_t>
  void optimizeProtections(graph_t& g, const std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*>& BFI) {
    llvm::dbgs() << "Optimizing protections in the graph\n";

    llvm::dbgs() << "Calculating hotness\n";
    auto rg = graph::filter::filter_removed_graph(g);

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
    }
  }
};
} // namespace composition::graph
#endif // COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
