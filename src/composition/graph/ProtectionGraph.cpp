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

vertex_idx_t ProtectionGraph::add_vertex(llvm::Value* v) {
  auto vFound = vertexCache.find(v);
  if (vFound != vertexCache.end() && VERTICES.find(vFound->second) != VERTICES.end()) {
    return vFound->second;
  }
  vertex_idx_t idx = VertexIdx++;

  auto vd = LG.addNode();
  (*vertices)[vd] = vertex_t(idx, v, llvmToVertexName(v), llvmToVertexType(v), {});
  // VERTICES.insert({idx, vd});
  VERTICES_DESCRIPTORS.insert({idx, vd});
  vertexCache.insert({v, idx});
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

edge_idx_t ProtectionGraph::add_edge(vertex_idx_t sIdx, vertex_idx_t dIdx, edge_t newE) {
  assert(sIdx != dIdx);
  auto source = VERTICES_DESCRIPTORS.left.at(sIdx);
  auto destination = VERTICES_DESCRIPTORS.left.at(dIdx);
  assert(source != destination);

  lemon::ListDigraph::Arc ed = lemon::findArc(LG, source, destination);
  if (ed != lemon::INVALID) {
    edge_idx_t idx = EDGES_DESCRIPTORS.right.at(ed);
    // edge_t& e = EDGES.at(idx);
    edge_t& e = (*edges)[ed];
    e.constraints.insert(newE.constraints.begin(), newE.constraints.end());
    EDGES_DESCRIPTORS.insert({idx, ed});
    return idx;
  }
  ed = LG.addArc(source, destination);

  edge_idx_t idx = newE.index;
  // EDGES.insert({idx, ed});
  (*edges)[ed] = newE;
  EDGES_DESCRIPTORS.insert({idx, ed});
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

void ProtectionGraph::removeManifest(manifest_idx_t idx) {
  for (auto [it, it_end] = MANIFESTS_CONSTRAINTS.left.equal_range(idx); it != it_end; ++it) {
    constraint_idx_t cIdx = it->second;

    auto vFound = CONSTRAINTS_VERTICES.find(cIdx);
    if (vFound != CONSTRAINTS_VERTICES.end()) {
      vertex_idx_t idx = vFound->second;
      vertex_t& v = VERTICES.at(idx);
      v.constraints.erase(cIdx);

      if (v.constraints.empty()) {
        remove_vertex(idx);
      }
    } else {
      auto eFound = CONSTRAINTS_EDGES.find(cIdx);
      if (eFound != CONSTRAINTS_EDGES.end()) {
        edge_idx_t idx = eFound->second;
        edge_t& e = EDGES.at(idx);
        e.constraints.erase(cIdx);

        if (e.constraints.empty()) {
          remove_edge(idx);
        }
      } else {
        llvm_unreachable("Constraint was neither a vertice nor an edge.");
      }
    }
  }

  for (auto [it, it_end] = DependencyUndo.right.equal_range(idx); it != it_end; ++it) {
    removeManifest(it->second);
  }
  DependencyUndo.right.erase(idx);
  // DependencyUndo.left.erase(idx);
  ManifestProtection.right.erase(idx);
  ManifestProtection.left.erase(idx);

  ManifestRegistry::Remove(MANIFESTS.find(idx)->second);
  MANIFESTS.erase(idx);
}

constraint_idx_t ProtectionGraph::addConstraint(manifest_idx_t idx, std::shared_ptr<Constraint> c) {
  CONSTRAINTS.insert({ConstraintIdx, c});
  MANIFESTS_CONSTRAINTS.insert({idx, ConstraintIdx});
  if (auto d = dyn_cast<Dependency>(c.get())) {
    auto dstNode = add_vertex(d->getFrom());
    auto srcNode = add_vertex(d->getTo());
    edge_idx_t idx = add_edge(srcNode, dstNode, edge_t{EdgeIdx++, edge_type::DEPENDENCY});
    addConstraintToEdge(ConstraintIdx, idx, c);
  } else if (auto present = dyn_cast<Present>(c.get())) {
    vertex_idx_t idx = add_vertex(present->getTarget());
    addConstraintToVertex(ConstraintIdx, idx, c);
  } else if (auto preserved = dyn_cast<Preserved>(c.get())) {
    vertex_idx_t idx = add_vertex(preserved->getTarget());
    addConstraintToVertex(ConstraintIdx, idx, c);
  } else if (auto tr = dyn_cast<True>(c.get())) {
    vertex_idx_t idx = add_vertex(tr->getTarget());
    addConstraintToVertex(ConstraintIdx, idx, c);
  } else {
    llvm_unreachable("Constraint unknown!");
  }
  return ConstraintIdx++;
}

void ProtectionGraph::addConstraintToVertex(constraint_idx_t ConstraintIdx, vertex_idx_t VertexIdx,
                                            std::shared_ptr<Constraint> c) {
  (*vertices)[VERTICES_DESCRIPTORS.left.at(VertexIdx)].constraints.insert({ConstraintIdx, c});
  CONSTRAINTS_VERTICES.insert({ConstraintIdx, VertexIdx});
}

void ProtectionGraph::addConstraintToEdge(constraint_idx_t ConstraintIdx, edge_idx_t EdgeIdx,
                                          std::shared_ptr<Constraint> c) {
  (*edges)[EDGES_DESCRIPTORS.left.at(EdgeIdx)].constraints.insert({ConstraintIdx, c});
  CONSTRAINTS_EDGES.insert({ConstraintIdx, EdgeIdx});
}

std::vector<Manifest*> ProtectionGraph::topologicalSortManifests(std::unordered_set<Manifest*> manifests) {
  std::set<Manifest*> all{manifests.begin(), manifests.end()};
  std::set<Manifest*> seen{};

  for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
    for (lemon::ListDigraph::InArcIt ei(LG, n); ei != lemon::INVALID; ++ei) {
      edge_idx_t idx = EDGES_DESCRIPTORS.right.at(ei);
      const edge_t& e = (*edges)[ei];
      for (auto& [cIdx, c] : e.constraints) {
        auto it = MANIFESTS_CONSTRAINTS.right.find(cIdx);
        if (it == MANIFESTS_CONSTRAINTS.right.end()) {
          llvm_unreachable("Protection was lost.");
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
  assert(lemon::checkedTopologicalSort(LG, order) == true);

  const int nodes = lemon::countNodes(LG);
  std::vector<lemon::ListDigraph::Node> sorted(nodes);

  for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
    sorted[order[n]] = static_cast<lemon::ListDigraph::Node>(n);
  }

  for (lemon::ListDigraph::Node& n : sorted) {
    for (lemon::ListDigraph::InArcIt ei(LG, n); ei != lemon::INVALID; ++ei) {
      edge_idx_t idx = EDGES_DESCRIPTORS.right.at(ei);
      const edge_t& e = (*edges)[ei];
      for (auto& [cIdx, c] : e.constraints) {
        manifest_idx_t idx = MANIFESTS_CONSTRAINTS.right.find(cIdx)->second;
        Manifest* manifest = MANIFESTS.find(idx)->second;

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

void ProtectionGraph::destroy() {
  vertexCache.clear();
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