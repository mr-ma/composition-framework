#include <array>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/graph/constraint/dependency.hpp>
#include <cstdint>
#include <lemon/connectivity.h>
#include <llvm/Support/raw_ostream.h>
#include <unordered_set>

using namespace llvm;
namespace composition::graph {

using graph::constraint::constraint_idx_t;
using graph::constraint::Dependency;
using support::cStats;

ProtectionGraph::ProtectionGraph() {
  vertices = std::make_unique<lemon::ListDigraph::NodeMap<vertex_t>>(LG);
  edges = std::make_unique<lemon::ListDigraph::ArcMap<edge_t>>(LG);
}

void ProtectionGraph::addManifests(std::vector<Manifest*> manifests) {
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

void ProtectionGraph::remove_vertex(vertex_idx_t idx) noexcept {
  /*graph_t& g = Graph;

  // vd_t vd = VERTICES_DESCRIPTORS.left.at(idx);
  for (auto [ei, ei_end] = boost::in_edges(vd, g); ei != ei_end; ++ei) {
    auto iFound = EDGES_DESCRIPTORS.right.find(*ei);
    if (iFound != EDGES_DESCRIPTORS.right.end()) {
      EDGES_DESCRIPTORS.right.erase(iFound);
    }
  }
  for (auto [ei, ei_end] = boost::out_edges(vd, g); ei != ei_end; ++ei) {
    auto iFound = EDGES_DESCRIPTORS.right.find(*ei);
    if (iFound != EDGES_DESCRIPTORS.right.end()) {
      EDGES_DESCRIPTORS.right.erase(iFound);
    }
  }
  boost::remove_vertex(vd, g);
  VERTICES.erase(idx);
  // VERTICES_DESCRIPTORS.left.erase(idx);*/
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

void ProtectionGraph::remove_edge(edge_idx_t idx) noexcept {
  /*
    for (auto [ei, ei_end] = EDGES_DESCRIPTORS.left.equal_range(idx); ei != ei_end; ++ei) {
      boost::remove_edge(ei->second, g);
    }
    EDGES.erase(idx);
    EDGES_DESCRIPTORS.left.erase(idx);
    */
}

void ProtectionGraph::removeManifest(manifest_idx_t mIdx) {
  /*for (auto [it, it_end] = MANIFESTS_CONSTRAINTS.left.equal_range(mIdx); it != it_end; ++it) {
    constraint_idx_t cIdx = it->second;

    if (auto vFound = CONSTRAINTS_VERTICES.find(cIdx); vFound != CONSTRAINTS_VERTICES.end()) {
      vertex_idx_t idx = vFound->second;
      vertex_t& v = (*vertices)[VERTICES_DESCRIPTORS.at(idx)];
      v.constraints.erase(cIdx);

      if (v.constraints.empty()) {
        remove_vertex(idx);
      }
    } else {
      auto eFound = CONSTRAINTS_EDGES.find(cIdx);
      if (eFound == CONSTRAINTS_EDGES.end()) {
        llvm_unreachable("Constraint was neither a vertice nor an edge.");
      }

      edge_idx_t idx = eFound->second;
      edge_t& e = (*edges)[EDGES_DESCRIPTORS.at(idx)];
      e.constraints.erase(cIdx);

      if (e.constraints.empty()) {
        remove_edge(idx);
      }
    }
  }

  for (auto [it, it_end] = DependencyUndo.right.equal_range(mIdx); it != it_end; ++it) {
    removeManifest(it->second);
  }
  DependencyUndo.right.erase(mIdx);
  // DependencyUndo.left.erase(idx);
  ManifestProtection.right.erase(mIdx);
  ManifestProtection.left.erase(mIdx);

  ManifestRegistry::Remove(MANIFESTS.find(mIdx)->second);
  MANIFESTS.erase(mIdx);*/
}

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

std::vector<Manifest*> ProtectionGraph::topologicalSortManifests(std::unordered_set<Manifest*> manifests) {
  lemon::ListDigraph::NodeMap<int> order{LG};
  assert(lemon::checkedTopologicalSort(LG, order) == true);

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
  // lemon::ListDigraph::NodeMap<int> order{LG};
  assert(lemon::checkedTopologicalSort(LG, order) == true);

  const unsigned long nodes = lemon::countNodes(LG);
  std::vector<lemon::ListDigraph::Node> sorted(nodes);

  for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
    sorted[nodes - order[n] - 1] = static_cast<lemon::ListDigraph::Node>(n);
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
  ManifestUndoMap undo{};
  for (auto& [idx, m] : MANIFESTS) {
    for (auto it : m->UndoValues()) {
      auto worked = undo.insert({idx, it});
      assert(worked.second && "undo");
    }
  }
  std::unordered_map<manifest_idx_t, std::unordered_set<llvm::Value*>> manifestUsers{};

  for (auto& [idx, u] : undo.left) {
    for (auto it = u->user_begin(), it_end = u->user_end(); it != it_end; ++it) {
      manifestUsers[idx].insert(*it);
    }
  }

  for (auto& [idx, u] : undo.left) {
    std::unordered_set<manifest_idx_t> manifests{};
    Manifest* m = MANIFESTS.find(idx)->second;

    for (auto I : m->Coverage()) {
      for (auto [it, it_end] = undo.right.equal_range(I); it != it_end; ++it) {
        manifests.insert(it->second);
      }
    }
    for (auto m2 : manifests) {
      ManifestProtection.insert({m2, idx});
    }
  }

  for (auto& [m, users] : manifestUsers) {
    for (auto u : users) {
      for (auto [it, it_end] = undo.right.equal_range(u); it != it_end; ++it) {
        if (m != it->second) {
          DependencyUndo.insert({it->second, m});
        }
      }
    }
  }
}

} // namespace composition::graph