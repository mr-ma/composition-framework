#include <algorithm>
#include <composition/graph/ILPSolver.hpp>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/graph/algorithm/all_cycles.hpp>
#include <composition/graph/constraint/dependency.hpp>
#include <composition/graph/constraint/present.hpp>
#include <composition/graph/constraint/preserved.hpp>
#include <composition/metric/ManifestStats.hpp>
#include <lemon/connectivity.h>
#include <queue>
#include <random>
#include <unordered_set>
#include <vector>

namespace composition::graph {
using composition::graph::ILPSolver;
using composition::graph::algorithm::AllCycles;
using composition::graph::constraint::Dependency;
using composition::graph::constraint::Present;
using composition::graph::constraint::PresentConstraint;
using composition::graph::constraint::Preserved;
using composition::graph::constraint::PreservedConstraint;
using composition::metric::Coverage;
using composition::metric::ManifestStats;
using llvm::dbgs;
using llvm::dyn_cast;

ProtectionGraph::ProtectionGraph() {
  vertices = std::make_unique<lemon::ListDigraph::NodeMap<vertex_t>>(LG);
  edges = std::make_unique<lemon::ListDigraph::ArcMap<edge_t>>(LG);
}

void ProtectionGraph::addManifests(std::set<Manifest*> manifests) {
  size_t total = manifests.size();
  dbgs() << "Adding " << std::to_string(total) << " manifests to protection graph\n";

  size_t i = 0;
  for (auto& m : manifests) {
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "\r";
    addManifest(m);
  }

  dbgs() << "#" << std::to_string(i) << "/" << std::to_string(total) << "\n";
}

void ProtectionGraph::addManifest(Manifest* m) {
  m->Clean();
  MANIFESTS.insert({m->index, m});
  for (auto& c : m->constraints) {
    addConstraint(m->index, c);
  }
}

vertex_idx_t ProtectionGraph::add_vertex(llvm::Value* value, bool shadow) {
  if (auto vFound = vertexCache[shadow]->find(value);
      vFound != vertexCache[shadow]->end() && VERTICES_DESCRIPTORS.find(vFound->second) != VERTICES_DESCRIPTORS.end()) {
    return vFound->second;
  }
  vertex_idx_t idx = VertexIdx++;

  auto vd = LG.addNode();
  (*vertices)[vd] = vertex_t(idx, value, llvmToVertexName(value), llvmToVertexType(value));
  if (shadow) {
    (*vertices)[vd].name += "_shadow";
  }
  VERTICES_DESCRIPTORS.insert({idx, vd});
  vertexCache[shadow]->insert({value, idx});
  return idx;
}

vertex_idx_t ProtectionGraph::add_vertex(
    llvm::Value* value,
    const std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>& constraints) {
  vertex_idx_t idx = add_vertex(value, false);
  auto& v = (*vertices)[VERTICES_DESCRIPTORS.at(idx)];

  for (auto& [cIdx, c] : constraints) {
    v.constraints.insert({cIdx, c});
    CONSTRAINTS_VERTICES.insert({cIdx, idx});
  }
  return idx;
}

edge_idx_t ProtectionGraph::add_edge(vertex_idx_t sIdx, vertex_idx_t dIdx) {
  assert(sIdx != dIdx);
  auto source = VERTICES_DESCRIPTORS.at(sIdx);
  auto destination = VERTICES_DESCRIPTORS.at(dIdx);
  assert(source != destination);

  if (auto ed = lemon::findArc(LG, source, destination); ed != lemon::INVALID) {
    return (*edges)[ed].index;
  }

  auto ed = LG.addArc(source, destination);
  edge_idx_t idx = EdgeIdx++;
  (*edges)[ed] = edge_t{idx};
  EDGES_DESCRIPTORS.insert({idx, ed});
  return idx;
}

edge_idx_t ProtectionGraph::add_edge(
    vertex_idx_t sIdx, vertex_idx_t dIdx,
    const std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>& constraints) {
  edge_idx_t idx = add_edge(sIdx, dIdx);
  auto& e = (*edges)[EDGES_DESCRIPTORS.at(idx)];

  for (auto& [cIdx, c] : constraints) {
    e.constraints.insert({cIdx, c});
    CONSTRAINTS_EDGES.insert({ConstraintIdx, EdgeIdx});
  }
  return idx;
}

size_t ProtectionGraph::countVertices() { return static_cast<size_t>(lemon::countNodes(LG)); }
size_t ProtectionGraph::countEdges() { return static_cast<size_t>(lemon::countArcs(LG)); }

constraint_idx_t ProtectionGraph::addConstraint(manifest_idx_t idx, std::shared_ptr<Constraint> c) {
  MANIFESTS_CONSTRAINTS.insert({idx, ConstraintIdx});
  if (auto d = dyn_cast<Dependency>(c.get())) {
    auto dstNode = add_vertex(d->getFrom(), true);
    auto srcNode = add_vertex(d->getTo(), false);
    add_edge(srcNode, dstNode, {{ConstraintIdx, c}});
  } else if (auto present = dyn_cast<Present>(c.get())) {
    add_vertex(present->getTarget(), {{ConstraintIdx, c}});
  } else if (auto preserved = dyn_cast<Preserved>(c.get())) {
    add_vertex(preserved->getTarget(), {{ConstraintIdx, c}});
  } else if (auto tr = dyn_cast<True>(c.get())) {
    add_vertex(tr->getTarget(), {{ConstraintIdx, c}});
  } else {
    llvm_unreachable("Constraint unknown!");
  }
  return ConstraintIdx++;
}

void addToConstraintsMaps(const vertex_t& v, PresentConstraint& present, PreservedConstraint& preserved,
                          std::set<constraint_idx_t>& presentConstraints,
                          std::set<constraint_idx_t>& preservedConstraints) {
  if (v.constraints.empty()) {
    return;
  }

  for (auto& [cIdx, c] : v.constraints) {
    if (auto* p1 = llvm::dyn_cast<Present>(c.get())) {
      presentConstraints.insert(cIdx);
      present = p1->isInverse() ? present | PresentConstraint::NOT_PRESENT : present | PresentConstraint::PRESENT;
    } else if (auto* p2 = llvm::dyn_cast<Preserved>(c.get())) {
      preservedConstraints.insert(cIdx);
      preserved =
          p2->isInverse() ? preserved | PreservedConstraint::NOT_PRESERVED : preserved | PreservedConstraint::PRESERVED;
    }
  }
}

std::set<std::pair<manifest_idx_t, manifest_idx_t>> ProtectionGraph::vertexConflicts() {
  std::set<std::pair<manifest_idx_t, manifest_idx_t>> conflicts{};

  for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
    PresentConstraint present = PresentConstraint::NONE;
    PreservedConstraint preserved = PreservedConstraint::NONE;

    std::set<constraint_idx_t> presentConstraints{};
    std::set<constraint_idx_t> preservedConstraints{};
    vertex_t& v = (*vertices)[n];
    addToConstraintsMaps(v, present, preserved, presentConstraints, preservedConstraints);

    auto addVertexConstraint = [&, this](vertex_idx_t idx) {
      vertex_t& v = (*vertices)[VERTICES_DESCRIPTORS.at(idx)];
      addToConstraintsMaps(v, present, preserved, presentConstraints, preservedConstraints);
    };

    auto addParentBB = [&, this](llvm::BasicBlock* BB) {
      if (BB->getParent() != nullptr) {
        llvm::Function* F = BB->getParent();
        if (auto vFound = vertexCache[false]->find(F); vFound != vertexCache[false]->end()) {
          addVertexConstraint(vFound->second);
        }
      }
    };

    auto addParentI = [&, this](llvm::Instruction* I) {
      if (I->getParent() != nullptr) {
        llvm::BasicBlock* BB = I->getParent();
        if (auto vFound = vertexCache[false]->find(BB); vFound != vertexCache[false]->end()) {
          addVertexConstraint(vFound->second);
          addParentBB(BB);
        }
      }
    };

    if (auto I = llvm::dyn_cast<llvm::Instruction>(v.value)) {
      addParentI(I);
    } else if (auto BB = llvm::dyn_cast<llvm::BasicBlock>(v.value)) {
      addParentBB(BB);
    }

    if (present == PresentConstraint::CONFLICT) {
      for (auto i = presentConstraints.begin(), i_end = presentConstraints.end(); i != i_end; ++i) {
        for (auto j = std::next(i, 1), j_end = presentConstraints.end(); j != j_end; ++j) {
          manifest_idx_t m1 = MANIFESTS_CONSTRAINTS.right.at(*i);
          manifest_idx_t m2 = MANIFESTS_CONSTRAINTS.right.at(*j);

          conflicts.insert({m1, m2});
        }
      }
    }

    if (preserved == PreservedConstraint::CONFLICT) {
      for (auto i = preservedConstraints.begin(), i_end = preservedConstraints.end(); i != i_end; ++i) {
        for (auto j = std::next(i, 1), j_end = preservedConstraints.end(); j != j_end; ++j) {
          manifest_idx_t m1 = MANIFESTS_CONSTRAINTS.right.at(*i);
          manifest_idx_t m2 = MANIFESTS_CONSTRAINTS.right.at(*j);

          conflicts.insert({m1, m2});
        }
      }
    }
  }

  return conflicts;
}

std::set<std::pair<manifest_idx_t, manifest_idx_t>> ProtectionGraph::computeDependencies() {
  std::set<std::pair<manifest_idx_t, manifest_idx_t>> dependencies{};

  for (auto& [mIdx, _] : MANIFESTS) {
    // Manifest dependencies
    for (auto [it, it_end] = DependencyUndo.right.equal_range(mIdx); it != it_end; ++it) {
      for (auto v : it->second) {
        dependencies.insert({v, mIdx});
      }
    }
  }

  return dependencies;
}
/*std::set<std::pair<manifest_idx_t,std::pair<manifest_idx_t, manifest_idx_t>>> ProtectionGraph::computeImplicitEdges()
{ std::set<std::pair<manifest_idx_t, manifest_idx_t>> dependencies{};

  for (auto& [mIdx, _] : MANIFESTS) {
    // Manifest dependencies
    for (auto [it, it_end] = DependencyUndo.right.equal_range(mIdx); it != it_end; ++it) {
      for (auto v : it->second) {
        dependencies.insert({v, mIdx});
      }
    }
  }

  return dependencies;
}*/

std::set<std::set<manifest_idx_t>> ProtectionGraph::computeCycles() {
  Profiler detectingProfiler{};

  if (lemon::dag(LG)) {
    return {};
  }

  std::set<std::set<manifest_idx_t>> cycles{};

  /*llvm::dbgs() << "Cycles...\n";
  AllCycles a{};
  llvm::dbgs() << "Nodes: " << lemon::countNodes(LG) << " Edges: " << lemon::countArcs(LG) << "\n";
  std::set<std::set<lemon::ListDigraph::Node>> all = a.simpleCycles(LG);
  llvm::dbgs() << "End...\n";*/

  lemon::ListDigraph::NodeMap<int> components{LG};
  lemon::stronglyConnectedComponents(LG, components);
  std::map<int, std::set<lemon::ListDigraph::Node>> sccs{};
  for (lemon::ListDigraph::NodeIt scNode(LG); scNode != lemon::INVALID; ++scNode) {
    sccs[components[scNode]].insert(scNode);
  }

  for (auto& [_, scc] : sccs) {
    if (scc.size() == 1) {
      continue;
    } else {
      std::set<manifest_idx_t> cycle{};

      for (auto& s : scc) {
        for (lemon::ListDigraph::OutArcIt e(LG, s); e != lemon::INVALID; ++e) {
          assert(LG.target(e) != s);
          if (scc.find(LG.target(e)) == scc.end()) {
            continue;
          }
          const edge_t& ed = (*edges)[e];
          for (auto& [cIdx, c] : ed.constraints) {
            if (auto mFound = MANIFESTS_CONSTRAINTS.right.find(cIdx); mFound != MANIFESTS_CONSTRAINTS.right.end()) {
              cycle.insert(mFound->second);
            }
          }
        }
        for (lemon::ListDigraph::InArcIt e(LG, s); e != lemon::INVALID; ++e) {
          assert(LG.source(e) != s);
          if (scc.find(LG.source(e)) == scc.end()) {
            continue;
          }
          const edge_t& ed = (*edges)[e];
          for (auto& [cIdx, c] : ed.constraints) {
            if (auto mFound = MANIFESTS_CONSTRAINTS.right.find(cIdx); mFound != MANIFESTS_CONSTRAINTS.right.end()) {
              cycle.insert(mFound->second);
            }
          }
        }
      }

      if (cycle.size() > 1) {
        cycles.insert(cycle);
      }
    }
  }
  cStats.timeConflictDetection += detectingProfiler.stop();

  return cycles;
}

std::set<std::set<manifest_idx_t>> ProtectionGraph::computeConnectivity(llvm::Module& M) {
  std::map<llvm::Instruction*, std::set<manifest_idx_t>> mapping{};

  for (auto& [mIdx, m] : MANIFESTS) {
    for (auto& I : m->Coverage()) {
      mapping[I].insert(mIdx);
    }
  }

  std::set<std::set<manifest_idx_t>> result{};
  for (auto& [I, mapped] : mapping) {
    result.insert(mapped);
  }

  return result;
}

std::set<std::set<manifest_idx_t>> ProtectionGraph::computeBlockConnectivity(llvm::Module& M) {
  std::map<llvm::BasicBlock*, std::set<manifest_idx_t>> mapping{};

  for (auto& [mIdx, m] : MANIFESTS) {
    auto blocks = m->BlockCoverage();
    for (auto& BB : blocks) {
      mapping[BB].insert(mIdx);
    }
  }

  std::set<std::set<manifest_idx_t>> result{};
  for (auto& [BB, mapped] : mapping) {
    result.insert(mapped);
  }

  return result;
}

size_t maxHotnessOfInstructions(std::set<llvm::Instruction*> instr,
                                const std::unordered_map<llvm::BasicBlock*, uint64_t>& BFI) {
  std::set<llvm::BasicBlock*> blocks = Coverage::InstructionsToBasicBlocks(instr);

  size_t maxHotness = 0;
  for (auto* B : blocks) {
    size_t newHotness = BFI.at(B);
    if (newHotness > maxHotness) {
      maxHotness = newHotness;
    }
  }
  return maxHotness;
}

size_t manifestHotness(Manifest* m, const std::unordered_map<llvm::BasicBlock*, uint64_t>& BFI) {
  std::set<llvm::Instruction*> instr = Coverage::ValuesToInstructions(m->UndoValues());
  return maxHotnessOfInstructions(instr, BFI);
}

size_t manifestHotnessProtectee(Manifest* m, const std::unordered_map<llvm::BasicBlock*, uint64_t>& BFI) {
  std::set<llvm::Instruction*> instr = m->Coverage();
  return maxHotnessOfInstructions(instr, BFI);
}

inline double round(double val) {
  if (val < 0)
    return ceil(val - 0.5);
  return floor(val + 0.5);
}

void ProtectionGraph::removeManifest(manifest_idx_t m) {
  std::stack<manifest_idx_t> s{};
  std::set<manifest_idx_t> processed{};
  s.push(m);

  while (!s.empty()) {
    manifest_idx_t current = s.top();
    s.pop();
    if (processed.find(current) != processed.end()) {
      continue;
    }
    processed.insert(current);
    if (DependencyUndo.right.find(current) != DependencyUndo.right.end()) {
      for (auto [it, it_end] = DependencyUndo.right.equal_range(current); it != it_end; ++it) {
        for (auto v : it->second) {
          s.push(v);
        }
      }
    }
    DependencyUndo.right.erase(current);
    ManifestRegistry::Remove(MANIFESTS.at(current));
    MANIFESTS.erase(current);
  }
}

std::set<Manifest*> ProtectionGraph::randomConflictHandling(llvm::Module& M) {
  std::set<Manifest*> accepted{};
  int cycleCount = 0;
  int conflictCount = 0;
  Profiler resolvingProfiler{};

  std::set<std::set<manifest_idx_t>> cycles;
  std::set<std::pair<manifest_idx_t, manifest_idx_t>> conflicts;

  auto RNG = std::mt19937{std::random_device{}()};
  do {
    accepted.clear();
    for (auto& [mIdx, m] : MANIFESTS) {
      accepted.insert(m);
    }

    ProtectionGraph pg{};
    pg.addManifests(accepted);
    pg.addHierarchy(M);
    pg.connectShadowNodes();

    cycles = pg.computeCycles();

    Profiler detectingProfiler{};
    conflicts = vertexConflicts();
    cStats.timeConflictDetection += detectingProfiler.stop();

    resolvingProfiler.reset();
    std::set<manifest_idx_t> flatConflicts{};

    for (auto& c : cycles) {
      ++cycleCount;
      for (auto& idx : c) {
        flatConflicts.insert(idx);
      }
    }
    for (auto& c : conflicts) {
      flatConflicts.insert(c.first);
      flatConflicts.insert(c.second);
      ++conflictCount;
    }
    if (!flatConflicts.empty()) {
      auto selected = *select_randomly(flatConflicts.begin(), flatConflicts.end());
      removeManifest(selected);
    }

    cStats.timeConflictResolving += resolvingProfiler.stop();
  } while (!cycles.empty() || !conflicts.empty());

  cStats.cycles = cycleCount;
  cStats.conflicts = conflictCount;

  return accepted;
}

std::unordered_set<llvm::Instruction*>
implictInstructions(manifest_idx_t idx, const ManifestProtectionMap& dep,
                    const std::map<manifest_idx_t, std::set<llvm::Instruction*>>& coverageCache) {

  std::unordered_set<manifest_idx_t> flatDeps{};

  std::unordered_set<manifest_idx_t> seen{};
  std::queue<manifest_idx_t> q{};

  for (auto [it, it_end] = dep.left.equal_range(idx); it != it_end; ++it) {
    for (auto v : it->second) {
      if (v == idx) {
        continue;
      }
      q.push(v);
      seen.insert(v);
    }
  }
  while (!q.empty()) {
    manifest_idx_t nextIdx = q.front();
    flatDeps.insert(nextIdx);
    q.pop();

    for (auto [it, it_end] = dep.left.equal_range(nextIdx); it != it_end; ++it) {
      for (auto v : it->second) {
        if (flatDeps.find(v) != flatDeps.end() || v == idx) {
          continue;
        }
        flatDeps.insert(v);
        q.push(v);
      }
    }
  }

  std::unordered_set<llvm::Instruction*> result{};
  for (auto d : flatDeps) {
    auto& cov = coverageCache.at(d);
    result.insert(cov.begin(), cov.end());
  }

  return result;
}

std::set<Manifest*> ProtectionGraph::ilpConflictHandling(llvm::Module& M,
                                                         const std::unordered_map<llvm::BasicBlock*, uint64_t>& BFI,
                                                         size_t totalInstructions) {
  size_t DesiredConnectivity = 2;
  size_t DesiredBlockConnectivity = 1;
  // TODO: Desired Implicit Coverage should be set by the cmd args
  double DesiredImplicitCoverage = 20;
  // TODO: cStats.stats is not set at this point ----
  size_t TotalInstructions = cStats.stats.numberOfAllInstructions;
  llvm::dbgs() << "Total instruction:" << totalInstructions << "\n";
  size_t implicitCoverageToInstruction = totalInstructions * (DesiredImplicitCoverage / 100);
  llvm::dbgs() << "Requested Implicit coverage of (instruction) " << implicitCoverageToInstruction << "\n";

  Profiler detectingProfiler{};
  auto conflicts = vertexConflicts();
  auto dependencies = computeDependencies();
  cStats.timeConflictDetection += detectingProfiler.stop();
  auto cycles = computeCycles();
  auto connectivities = computeConnectivity(M);
  auto blockConnectivities = computeBlockConnectivity(M);

  // Sanity check
  assert(Performance::hasProfiling(M));

  // Prepare manifest statistics
  std::map<manifest_idx_t, ManifestStats> eStats{};

  std::map<manifest_idx_t, ManifestStats> mStats{};
  std::map<manifest_idx_t, std::set<llvm::Instruction*>> coverageCache{};
  std::pair<size_t, size_t> explicitCBounds{SIZE_MAX, 0};
  std::pair<size_t, size_t> implicitCBounds{SIZE_MAX, 0};
  std::pair<size_t, size_t> hotnessBounds{SIZE_MAX, 0};
  std::pair<size_t, size_t> hotnessProtecteeBounds{SIZE_MAX, 0};

  for (auto& [mIdx, m] : MANIFESTS) {
    auto cov = m->Coverage();
    coverageCache.insert({mIdx, cov});
    mStats[mIdx].explicitC = cov.size();
    mStats[mIdx].hotness = manifestHotness(m, BFI);
    mStats[mIdx].hotnessProtectee = manifestHotnessProtectee(m, BFI);

    explicitCBounds.first = std::min(explicitCBounds.first, mStats[mIdx].explicitC);
    explicitCBounds.second = std::max(explicitCBounds.second, mStats[mIdx].explicitC);

    hotnessBounds.first = std::min(hotnessBounds.first, mStats[mIdx].hotness);
    hotnessBounds.second = std::max(hotnessBounds.second, mStats[mIdx].hotness);

    hotnessProtecteeBounds.first = std::min(hotnessProtecteeBounds.first, mStats[mIdx].hotnessProtectee);
    hotnessProtecteeBounds.second = std::max(hotnessProtecteeBounds.second, mStats[mIdx].hotnessProtectee);
  }
  std::map<manifest_idx_t /*protected manifest*/, std::pair<std::set<manifest_idx_t> /*edges*/, unsigned long>>
      duplicateEdgesOnManifest{};
  metric::Stats s{};
  //std::vector<std::tuple<manifest_idx_t /*edge_index*/, std::pair<manifest_idx_t, manifest_idx_t> /*m1 -> m2*/,
  //                       unsigned long /*coverage*/>>
  auto implicitCov = s.implictInstructionsPerEdge(ManifestProtection, MANIFESTS, &duplicateEdgesOnManifest);

  for (auto [edgeIndex, manifestPair, coverage] : implicitCov) {
    eStats[edgeIndex].implicitC = coverage;
    implicitCBounds.first = std::min(implicitCBounds.first, coverage);
    implicitCBounds.second = std::max(implicitCBounds.second, coverage);
  }

  for (auto& [idx, s] : mStats) {
    s.normalize(explicitCBounds, implicitCBounds, hotnessBounds, hotnessProtecteeBounds);
  }

  Profiler resolvingProfiler{};

  auto costFunction = [](ManifestStats s) -> double {
    return 1.0 * s.normalizedHotness + (1.0 - s.normalizedHotnessProtectee);
  };
  do {
    ILPSolver solver{};
    solver.init(0, implicitCoverageToInstruction, 0, 0);
    solver.DesiredConnectivity = DesiredConnectivity;
    solver.DesiredBlockConnectivity = DesiredBlockConnectivity;
    solver.setCostFunction(costFunction);
    solver.addManifests(MANIFESTS, mStats);
    solver.addDependencies(dependencies);
    solver.addConflicts(conflicts);
    solver.addCycles(cycles);
    solver.addConnectivity(connectivities);
    solver.addBlockConnectivity(blockConnectivities);
    solver.addImplicitCoverage(implicitCov, duplicateEdgesOnManifest);
    auto [acceptedIndices, acceptedEdges] = solver.run();
    solver.destroy();

    std::set<Manifest*> accepted{};
    for (auto& mIdx : acceptedIndices) {
      accepted.insert(MANIFESTS.at(mIdx));
      // TODO: calculate implicit coverage based on the accepted edges
    }
    for (auto& eIdx : acceptedEdges) {
      // accepted.insert(MANIFESTS.at(mIdx));
      // TODO: calculate implicit coverage based on the accepted edges
      llvm::dbgs() << "accepted edge" << eIdx << "\n";
    }

    ProtectionGraph pg{};
    pg.addManifests(accepted);
    pg.addHierarchy(M);
    pg.connectShadowNodes();
    auto newCycles = pg.computeCycles();
    if (!newCycles.empty()) {
      for (auto& c : newCycles) {
        cycles.insert(c);
      }
    } else {
      cStats.cycles = cycles.size();
      cStats.conflicts = conflicts.size();
      cStats.timeConflictResolving += resolvingProfiler.stop();
      return accepted;
    }
  } while (true);
}

std::vector<Manifest*> ProtectionGraph::topologicalSortManifests(std::set<Manifest*> manifests) {
  std::set<Manifest*> all{manifests.begin(), manifests.end()};
  std::set<Manifest*> seen{};

  for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
    for (lemon::ListDigraph::InArcIt ei(LG, n); ei != lemon::INVALID; ++ei) {
      const edge_t& e = (*edges)[ei];
      for (auto& [cIdx, c] : e.constraints) {
        auto it = MANIFESTS_CONSTRAINTS.right.find(cIdx);
        if (it == MANIFESTS_CONSTRAINTS.right.end()) {
          continue;
        }
        manifest_idx_t idx = it->second;
        seen.insert(MANIFESTS.find(idx)->second);
      }
    }
  }

  std::vector<Manifest*> result{};
  std::set_difference(all.begin(), all.end(), seen.begin(), seen.end(), std::back_inserter(result));

  seen.clear();
  lemon::ListDigraph::NodeMap<int> order{LG};
  assert(lemon::checkedTopologicalSort(LG, order));

  const auto nodes = static_cast<unsigned long>(lemon::countNodes(LG));
  std::vector<lemon::ListDigraph::Node> sorted(nodes);

  for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
    sorted[order[n]] = n;
  }

  for (lemon::ListDigraph::Node& n : sorted) {
    for (lemon::ListDigraph::InArcIt ei(LG, n); ei != lemon::INVALID; ++ei) {
      const edge_t& e = (*edges)[ei];
      edge_idx_t idx = e.index;
      for (auto& [cIdx, c] : e.constraints) {
        auto mFound = MANIFESTS_CONSTRAINTS.right.find(cIdx);
        if (mFound == MANIFESTS_CONSTRAINTS.right.end()) {
          continue;
        }
        Manifest* manifest = MANIFESTS.at(mFound->second);

        if (seen.find(manifest) != seen.end()) {
          continue;
        }
        seen.insert(manifest);
        result.push_back(manifest);
      }
    }
  }

  return result;
}

void ProtectionGraph::addHierarchy(llvm::Module& M) {
  for (auto&& F : M) {
    auto fNode = add_vertex(&F, false);
    for (auto&& BB : F) {
      auto bbNode = add_vertex(&BB, false);
      add_edge(bbNode, fNode, {{ConstraintIdx++, std::make_shared<Dependency>("hierarchy", &BB, &F)}});
      for (auto&& I : BB) {
        auto iNode = add_vertex(&I, false);
        add_edge(iNode, bbNode, {{ConstraintIdx++, std::make_shared<Dependency>("hierarchy", &I, &BB)}});
      }
    }
  }
}

void ProtectionGraph::connectShadowNodes() {
  for (const auto& [value, sIdx] : vertexShadowCache) {
    add_edge(sIdx, add_vertex(value, false));
    if (auto* F = llvm::dyn_cast<llvm::Function>(value)) {
      for (auto&& BB : *F) {
        add_edge(sIdx, add_vertex(&BB, false));
        for (auto&& I : BB) {
          add_edge(sIdx, add_vertex(&I, false));
        }
      }
    } else if (auto* BB = llvm::dyn_cast<llvm::BasicBlock>(value)) {
      for (auto&& I : *BB) {
        add_edge(sIdx, add_vertex(&I, false));
      }
    }
  }
}

void ProtectionGraph::destroy() {
  vertexRealCache.clear();
  vertexShadowCache.clear();
  DependencyUndo.clear();
}

void ProtectionGraph::computeManifestDependencies() {
  // Find all the llvm::Values that where added by a manifest
  composition::util::bimap<manifest_idx_t, llvm::Value*> undo{};
  for (auto& [idx, m] : MANIFESTS) {
    for (llvm::Value* it : m->UndoValues()) {
      undo.insert({idx, it});
    }
  }

  // Construct the protection relationship between manifests
  // Loop over the manifests
  for (auto& [idx, m] : MANIFESTS) {
    // Take the coverage of the manifests
    for (auto I : m->Coverage()) {
      // Check if the covered llvm::Value is a guard of a different manifest
      for (auto [it, it_end] = undo.right.equal_range(I); it != it_end; ++it) {
        for (auto v : it->second) {
          if (v != idx) {
            // Then add an edge between both manifests
            ManifestProtection.insert({v, idx});
          }
        }
      }
    }
  }

  // For each manifest M find its dependent manifests.
  // A dependent manifest must be removed if M is removed.
  // First find the users
  std::unordered_map<manifest_idx_t, std::unordered_set<llvm::Value*>> manifestUsers{};
  for (auto& [idx, values] : undo.left) {
    for (auto u : values) {
      for (auto it = u->user_begin(), it_end = u->user_end(); it != it_end; ++it) {
        manifestUsers[idx].insert(*it);
      }
    }
  }

  // Then construct the dependency
  for (auto& [m, users] : manifestUsers) {
    for (auto u : users) {
      for (auto [it, it_end] = undo.right.equal_range(u); it != it_end; ++it) {
        for (auto v : it->second) {
          if (m != v) {
            DependencyUndo.insert({v, m});
          }
        }
      }
    }
  }
}

} // namespace composition::graph
