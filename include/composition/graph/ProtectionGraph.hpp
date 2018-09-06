#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#include <utility>
#include <random>
#include <algorithm>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/copy.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/graph.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/graph/algorithm/strong_components.hpp>
#include <composition/graph/algorithm/topological_sort.hpp>
#include <composition/graph/util/index_map.hpp>
#include <composition/graph/util/vertex_count.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/filter/dependency.hpp>
#include <composition/graph/filter/removed.hpp>
#include <composition/graph/filter/selfcycle.hpp>
#include <composition/options.hpp>
#include <composition/strategy/Strategy.hpp>
#include <composition/strategy/Random.hpp>
#include <composition/profiler.hpp>

namespace composition {
using ProtectionIndex = unsigned long;
using VertexIndex = uintptr_t;
using EdgeIndex = uintptr_t;
using ProtectionMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, ProtectionIndex>;

//using ManifestCoverageMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, boost::bimaps::multiset_of<llvm::Instruction*>>;
//using ProtecteeManifestMap = boost::bimaps::bimap<boost::bimaps::multiset_of<llvm::Instruction*>, std::shared_ptr<Manifest>>;
using ManifestUndoMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, boost::bimaps::multiset_of<llvm::Value*>>;
using ManifestDependencyMap = boost::bimaps::bimap<boost::bimaps::multiset_of<std::shared_ptr<Manifest>>, boost::bimaps::multiset_of<std::shared_ptr<Manifest>>>;


class ProtectionGraph {
private:
  graph_t Graph{};
  ProtectionIndex ProtectionIdx{};
  ProtectionMap Protections{};
  std::unordered_map<VertexIndex, vd_t> vertexCache{};
  ManifestDependencyMap DependencyUndo{};

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

  ProtectionGraph(ProtectionGraph &that) = delete;

  ProtectionGraph(ProtectionGraph &&that) noexcept : ProtectionIdx(that.ProtectionIdx),
                                                     Protections(that.Protections),
                                                     vertexCache(that.vertexCache) {
    auto index = index_map(that.Graph);
    boost::copy_graph(that.Graph, this->Graph, vertex_index_map(boost::make_assoc_property_map(index)));
  }

  ProtectionGraph &operator=(ProtectionGraph &that) = delete;

  ProtectionGraph &operator=(ProtectionGraph &&that) = default;

  void destroy();

  graph_t &getGraph();

  ProtectionIndex addConstraint(std::shared_ptr<Manifest> m, std::shared_ptr<Constraint> c);

  template<typename T, typename S>
  ProtectionIndex addCFG(T parent, S child) {
    assert(parent != nullptr);
    assert(child != nullptr);
    auto srcNode = this->add_vertex(parent);
    auto dstNode = this->add_vertex(child);
    this->add_edge(srcNode, dstNode, edge_t{ProtectionIdx, "CFG", edge_type::CFG});
    return ProtectionIdx++;
  }

  std::vector<std::shared_ptr<Manifest>> topologicalSortManifests(std::vector<std::shared_ptr<Manifest>> manifests);

  void removeManifest(std::shared_ptr<Manifest> m);

  void expandToInstructions();

  void reduceToInstructions();

  void expandToFunctions();

  void reduceToFunctions();

  void computeManifestDependencies();

  template<typename T>
  void conflictHandling(T &g, const std::unique_ptr<Strategy> &strategy) {
    llvm::dbgs() << "Step 1: Removing cycles...\n";
    auto rg = filter_removed_graph(g);
    auto fg = filter_dependency_graph(rg);

    //First detect cycles and remove all cycles.
    bool hadConflicts;
    Profiler sccProfiler{};
    Profiler resolvingProfiler{};
    do {
      hadConflicts = false;
      sccProfiler.reset();
      auto components = strong_components(fg);
      cStats.timeCycleDetection += sccProfiler.stop();
      int i = 0;
      for (auto &component : components) {
        auto vertexCount = vertex_count(component);
        if (vertexCount != 1) {
          hadConflicts = true;
          llvm::dbgs() << "Component " << std::to_string(i) << " contains cycle with " << std::to_string(vertexCount)
                       << " elements.\n";
          if (DumpGraphs) {
            save_graph_to_dot(component, "graph_component_" + std::to_string(i) + ".dot");
            save_graph_to_graphml(component, "graph_component_" + std::to_string(i) + ".graphml");
          }
          resolvingProfiler.reset();
          handleCycle(component, strategy);
          cStats.timeConflictResolving += resolvingProfiler.stop();
          ++i;
        }
      }
    } while (hadConflicts);

    llvm::dbgs() << "Step 2: Removing remaining present/preserved conflicts...\n";
    //Then remove remaining present/preserved conflicts.
    do {
      hadConflicts = false;

      auto[presentManifests, preservedManifests] = detectPresentPreservedConflicts(g);
      if (!presentManifests.empty() || !preservedManifests.empty()) {
        hadConflicts = true;
        llvm::dbgs() << "Handling conflict...\n";

        resolvingProfiler.reset();
        std::vector<std::shared_ptr<Manifest>> merged{};
        std::set_union(presentManifests.begin(), presentManifests.end(),
                       preservedManifests.begin(), preservedManifests.end(),
                       std::back_inserter(merged));

        removeManifest(strategy->decidePresentPreserved(merged));
        cStats.timeConflictResolving += resolvingProfiler.stop();
      }
    } while (hadConflicts);
  }

  template<typename graph_t>
  void handleCycle(graph_t &g, const std::unique_ptr<Strategy> &strategy) {
    auto[presentManifests, preservedManifests] = detectPresentPreservedConflicts(g);

    llvm::dbgs() << "Handling cycle in component\n";
    for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
      auto v = g[*vi];

      //llvm::dbgs() << v.name << "\n";
      //llvm::dbgs() << std::to_string(v.index) << "\n";
    }

    std::vector<std::shared_ptr<Manifest>> cyclicManifests{};
    for (auto[ei, ei_end] = boost::edges(g); ei != ei_end; ++ei) {
      cyclicManifests.push_back(Protections.right.find(g[*ei].index)->second);
    }

    std::set<std::shared_ptr<Manifest>> pre_merged{};
    std::set_union(presentManifests.begin(), presentManifests.end(),
                   preservedManifests.begin(), preservedManifests.end(),
                   std::inserter(pre_merged, pre_merged.begin()));

    std::vector<std::shared_ptr<Manifest>> merged{};
    std::set_union(pre_merged.begin(), pre_merged.end(),
                   cyclicManifests.begin(), cyclicManifests.end(),
                   std::back_inserter(merged));

    removeManifest(strategy->decideCycle(merged));
  }

  template<typename graph_t>
  std::pair<std::set<std::shared_ptr<Manifest>>, std::set<std::shared_ptr<Manifest>>> detectPresentPreservedConflicts(
      graph_t &g) {
    auto[isPresent, isPreserved] = constraint_map<PresentConstraint, PreservedConstraint>(g);

    std::set<std::shared_ptr<Manifest>> presentManifests{};
    for (auto[vd, p] : isPresent) {
      if (p != PresentConstraint::CONFLICT) {
        continue;
      }

      for (auto[index, c] : g[vd].constraints) {
        if (llvm::isa<Present>(c.get())) {
          presentManifests.insert(Protections.right.find(index)->second);
        }
      }
    }

    std::set<std::shared_ptr<Manifest>> preservedManifests{};
    for (auto[vd, p] : isPreserved) {
      if (p != PreservedConstraint::CONFLICT) {
        continue;
      }
      for (auto[index, c] : g[vd].constraints) {
        if (llvm::isa<Preserved>(c.get())) {
          preservedManifests.insert(Protections.right.find(index)->second);
        }
      }
    }

    return {presentManifests, preservedManifests};
  }
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
