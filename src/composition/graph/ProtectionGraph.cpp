#include <composition/graph/ProtectionGraph.hpp>
#include <composition/graph/constraint/dependency.hpp>
#include <cstdint>
#include <llvm/Support/raw_ostream.h>
#include <unordered_set>

using namespace llvm;
namespace composition::graph {

using graph::algorithm::reverse_topological_sort;
using graph::constraint::constraint_idx_t;
using graph::constraint::Dependency;
using graph::filter::filter_dependency_graph;
using graph::filter::filter_removed_graph;
using graph::filter::filter_selfcycle_graph;
using support::cStats;

graph_t& ProtectionGraph::getGraph() { return Graph; }

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

vd_t ProtectionGraph::add_vertex(llvm::Value* v) {
  graph_t& g = Graph;

  if (vertexCache.find(v) != vertexCache.end()) {
    auto vd = vertexCache.at(v);
    g[vd].removed = false;
    return vd;
  }

  auto vd = boost::add_vertex(g);
  auto idx = VertexIdx++;
  g[vd] = vertex_t(idx, v, llvmToVertexName(v), llvmToVertexType(v), {});
  vertexCache.insert({v, vd});
  return vd;
}

void ProtectionGraph::remove_vertex(vd_t vd) noexcept {
  graph_t& g = Graph;
  g[vd].removed = true;
  for (auto [ei, ei_end] = boost::in_edges(vd, g); ei != ei_end; ++ei) {
    remove_edge(*ei);
  }
  for (auto [ei, ei_end] = boost::out_edges(vd, g); ei != ei_end; ++ei) {
    remove_edge(*ei);
  }
}

ed_t ProtectionGraph::add_edge(vd_t s, vd_t d, edge_t newE) {
  graph_t& g = Graph;
  auto edge = boost::add_edge(s, d, g);
  edge_t &e = g[edge.first];
  if(!edge.second) {
    e.removed = false;
    e.constraints.insert(newE.constraints.begin(), newE.constraints.end());
  } else {
    e = std::move(newE);
  }
  return edge.first;
}

void ProtectionGraph::remove_edge(ed_t ed) noexcept {
  graph_t& g = Graph;
  g[ed].removed = true;
}

void ProtectionGraph::removeManifest(manifest_idx_t idx) {
  graph_t& g = Graph;

  for(auto [it, it_end] = MANIFESTS_CONSTRAINTS.left.equal_range(idx); it != it_end; ++it) {
    constraint_idx_t cIdx = it->second;

    auto vFound = CONSTRAINTS_VERTICES.find(cIdx);
    if(vFound != CONSTRAINTS_VERTICES.end()) {
      vd_t v = vFound->second;
      g[v].constraints.erase(cIdx);

      if(g[v].constraints.empty()) {
        remove_vertex(v);
      }
    } else {
      auto eFound = CONSTRAINTS_EDGES.find(cIdx);
      if(eFound != CONSTRAINTS_EDGES.end()) {
        ed_t e = eFound->second;
        g[e].constraints.erase(cIdx);

        if(g[e].constraints.empty()) {
          remove_edge(e);
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

void ProtectionGraph::expandToInstructions() {
  // 1) Loop over all nodes
  // 2) If node is not an instruction -> Resolve instructions and add to graph
  // 3) Add edge for all edges to instructions
  graph_t& g = Graph;

  size_t total =  boost::num_vertices(g);
  dbgs() << "Transforming " << std::to_string(total) << " vertices to instructions\n";
  size_t i = 0;  

  for (auto [vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "; " <<  boost::num_vertices(g) << "-" << boost::num_edges(g) << "\r";

    vertex_t &v = g[*vi];

    switch (v.type) {
    case vertex_type::FUNCTION: {
      auto func = static_cast<llvm::Function*>(v.value);
      for (auto& B : *func) {
        this->expandBasicBlockToInstructions(*vi, &B);
      }
    } break;
    case vertex_type::BASICBLOCK: {
      auto B = static_cast<llvm::BasicBlock*>(v.value);
      this->expandBasicBlockToInstructions(*vi, B);
    } break;
    case vertex_type::INSTRUCTION:
      break;
    case vertex_type::VALUE:
      break;
    case vertex_type::UNKNOWN:
      break;
    }
  }
  dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "; " <<  boost::num_vertices(g) << "-" << boost::num_edges(g) << "\n";

  cStats.vertices = boost::num_vertices(g);
  cStats.edges = boost::num_edges(g);
}

void ProtectionGraph::expandBasicBlockToInstructions(vd_t it, llvm::BasicBlock* B) {
  graph_t& g = Graph;

  for (auto& I : *B) {
    auto node = this->add_vertex(&I);
    auto &constraints = g[it].constraints;
    g[node].constraints.insert(constraints.begin(), constraints.end());
    replaceTarget(it, node);
  }
}

void ProtectionGraph::reduceToInstructions() {
  graph_t& g = Graph;

  for (auto [vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    vertex_t& v = g[*vi];
    switch (v.type) {
    case vertex_type::FUNCTION:
      remove_vertex(*vi);
      break;
    case vertex_type::BASICBLOCK:
      remove_vertex(*vi);
      break;
    case vertex_type::INSTRUCTION: {
      if (boost::out_degree(*vi, g) == 0 && boost::in_degree(*vi, g) == 0 && g[*vi].constraints.empty()) {
        remove_vertex(*vi);
      }
      break;
    }
    case vertex_type::VALUE:
      break;
    case vertex_type::UNKNOWN:
      break;
    }
  }
}

void ProtectionGraph::replaceTarget(vd_t src, vd_t dst) {
  replaceTargetInEdges(src, dst);
  replaceTargetOutEdges(src, dst);
}

void ProtectionGraph::replaceTargetInEdges(vd_t src, vd_t dst) {
  graph_t& g = Graph;

  for (auto [vi, vi_end] = boost::in_edges(src, g); vi != vi_end; ++vi) {
    edge_t e = g[*vi];
    if (e.type != edge_type::DEPENDENCY) {
      continue;
    }

    auto from = boost::source(*vi, g);
    this->add_edge(from, dst, edge_t{e.index, e.type});
  }
}

void ProtectionGraph::replaceTargetOutEdges(vd_t src, vd_t dst) {
  graph_t& g = Graph;

  for (auto [vi, vi_end] = boost::out_edges(src, g); vi != vi_end; ++vi) {
    edge_t e = g[*vi];
    if (e.type != edge_type::DEPENDENCY) {
      continue;
    }

    auto to = boost::target(*vi, g);
    this->add_edge(dst, to, edge_t{e.index, e.type});
  }
}

constraint_idx_t ProtectionGraph::addConstraint(manifest_idx_t idx, std::shared_ptr<Constraint> c) {
  graph_t& g = Graph;

  CONSTRAINTS.insert({ConstraintIdx, c});
  if (auto d = dyn_cast<Dependency>(c.get())) {
    auto dstNode = this->add_vertex(d->getFrom());
    auto srcNode = this->add_vertex(d->getTo());
    ed_t e = this->add_edge(srcNode, dstNode, edge_t{EdgeIdx++, edge_type::DEPENDENCY});
    CONSTRAINTS_EDGES.insert({ConstraintIdx, e});
  } else if (auto present = dyn_cast<Present>(c.get())) {
    vd_t v = this->add_vertex(present->getTarget());
    g[v].constraints.insert({ConstraintIdx, c});
    CONSTRAINTS_VERTICES.insert({ConstraintIdx, v});
  } else if (auto preserved = dyn_cast<Preserved>(c.get())) {
    vd_t v = this->add_vertex(preserved->getTarget());
    g[v].constraints.insert({ConstraintIdx, c});
    CONSTRAINTS_VERTICES.insert({ConstraintIdx, v});
  } else if (auto tr = dyn_cast<True>(c.get())) {
    vd_t v = this->add_vertex(tr->getTarget());
    g[v].constraints.insert({ConstraintIdx, c});
    CONSTRAINTS_VERTICES.insert({ConstraintIdx, v});
  } else {
    llvm_unreachable("Constraint unknown!");
  }
  return ConstraintIdx++;
}

std::vector<Manifest*> ProtectionGraph::topologicalSortManifests(std::unordered_set<Manifest*> manifests) {
  auto rg = filter_removed_graph(Graph);
  auto fg = filter_dependency_graph(rg);
  auto sc = filter_selfcycle_graph(fg);

  std::set<Manifest*> all{manifests.begin(), manifests.end()};
  std::set<Manifest*> seen{};
  for (auto [vi, vi_end] = boost::vertices(sc); vi != vi_end; ++vi) {
    for (auto [ei, ei_end] = boost::in_edges(*vi, sc); ei != ei_end; ++ei) {
      edge_t e = sc[*ei];
      for(auto [cIdx, c] : e.constraints) {
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
  auto sorted = reverse_topological_sort(sc);
  for (auto v : sorted) {
    for (auto [ei, ei_end] = boost::in_edges(v, sc); ei != ei_end; ++ei) {
      edge_t e = sc[*ei];
      for(auto [cIdx, c] : e.constraints) {
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
  Graph = {};
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