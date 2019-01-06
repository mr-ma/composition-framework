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
#include <composition/graph/constraint/true.hpp>
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
#include <type_traits>
#include <utility>

namespace composition::graph {
using graph::algorithm::strong_components;
using graph::algorithm::topological_sort;
using graph::constraint::Constraint;
using graph::constraint::constraint_idx_t;
using graph::constraint::Present;
using graph::constraint::PresentConstraint;
using graph::constraint::Preserved;
using graph::constraint::PreservedConstraint;
using graph::constraint::True;
using graph::util::graph_to_dot;
using graph::util::graph_to_graphml;
using metric::Performance;
using support::cStats;
using util::constraint_map;
using util::index_map;
using util::vertex_count;

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
  graph_t Graph{};
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
  std::unordered_map<llvm::Value*, vd_t> vertexCache{};
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

  std::unordered_map<constraint_idx_t, std::shared_ptr<Constraint>> CONSTRAINTS;
  boost::bimaps::bimap<boost::bimaps::multiset_of<manifest_idx_t>, constraint_idx_t> MANIFESTS_CONSTRAINTS;
  std::unordered_map<constraint_idx_t, vd_t> CONSTRAINTS_VERTICES;
  std::unordered_map<constraint_idx_t, ed_t> CONSTRAINTS_EDGES;

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

  void addManifests(std::vector<Manifest*> manifests);
  void addManifest(Manifest* m);

  /**
   * Adds a constraint to the protection graph
   * @param m the manifest associated with the constraint
   * @param c the constraint
   * @return a unique protection index
   */
  constraint_idx_t addConstraint(manifest_idx_t idx, std::shared_ptr<Constraint> c);

  /**
   * Adds CFG edges to the graph
   * @param parent the source llvm value
   * @param child the target llvm value
   * @return a unique protection index
   */
  edge_idx_t addCFG(llvm::Value* parent, llvm::Value* child) {
    assert(parent != nullptr);
    assert(child != nullptr);
    auto srcNode = this->add_vertex(parent);
    auto dstNode = this->add_vertex(child);
    this->add_edge(srcNode, dstNode, edge_t{EdgeIdx, edge_type::CFG});
    return EdgeIdx++;
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
  void removeManifest(manifest_idx_t idx);

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

        removeManifest(strategy->decidePresentPreserved(merged)->index);
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

    std::unordered_set<manifest_idx_t> manifestIdx{};
    for (auto [ei, ei_end] = boost::edges(g); ei != ei_end; ++ei) {
      edge_t e = g[*ei];
      for(auto [cIdx, c] : e.constraints) {
        manifestIdx.insert(MANIFESTS_CONSTRAINTS.right.find(cIdx)->second);
      }
    }

    std::vector<Manifest*> cyclicManifests{};
    for (auto& el : manifestIdx) {
      cyclicManifests.push_back(MANIFESTS.find(el)->second);
    }

    removeManifest(strategy->decideCycle(cyclicManifests)->index);
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

    std::unordered_set<manifest_idx_t> presentIdx{};
    for (auto [vd, p] : isPresent) {
      if (p != PresentConstraint::CONFLICT) {
        continue;
      }

      for (auto [cIdx, c] : g[vd].constraints) {
        if (llvm::isa<Present>(c.get())) {
          manifest_idx_t idx = MANIFESTS_CONSTRAINTS.right.find(cIdx)->second;
          presentIdx.insert(idx);
        }
      }
    }

    std::unordered_set<manifest_idx_t> preservedIdx{};
    for (auto [vd, p] : isPreserved) {
      if (p != PreservedConstraint::CONFLICT) {
        continue;
      }
      for (auto [cIdx, c] : g[vd].constraints) {
        if (llvm::isa<Preserved>(c.get())) {
          manifest_idx_t idx = MANIFESTS_CONSTRAINTS.right.find(cIdx)->second;
          preservedIdx.insert(idx);
        }
      }
    }

    //Convert indices to manifest pointers
    std::unordered_set<Manifest*> presentManifests{};
    std::unordered_set<Manifest*> preservedManifests{};
    for(auto idx : presentIdx) {
      presentManifests.insert(MANIFESTS.find(idx)->second);
    }
    for(auto idx : preservedIdx) {
      preservedManifests.insert(MANIFESTS.find(idx)->second);
    }

    return {presentManifests, preservedManifests};
  }

  template <typename graph_t>
  void optimizeProtections(graph_t& g, const std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*>& BFI,
                           llvm::Module* module, std::vector<Manifest*> manifests,
                           std::unordered_set<llvm::Instruction*> allInstructions) {
    llvm::dbgs() << "Optimizing protections in the graph\n";
    prepareHotness(g, BFI);
    removeHotNodes(g, 1.0, module, manifests, allInstructions);
  }

  template <typename graph_t>
  void prepareHotness(graph_t& g, const std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*>& BFI) {
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

  template <typename graph_t>
  void removeHotNodes(graph_t& g, double coverage, llvm::Module* module, std::vector<Manifest*> manifests,
                      std::unordered_set<llvm::Instruction*> allInstructions) {
    llvm::dbgs() << "Removing hot nodes\n";

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
      auto rg = graph::filter::filter_removed_graph(g);
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
      /*for (auto mi = Protections.left.begin(), mi_end = Protections.left.end(); mi != mi_end; ++mi) {
        manifests.push_back(MANIFESTS.find(mi->first)->second);
      }*/
    }
  }
};
} // namespace composition::graph
#endif // COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
