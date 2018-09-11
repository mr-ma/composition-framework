#include <cstdint>
#include <unordered_set>
#include <llvm/Support/raw_ostream.h>
#include <composition/graph/ProtectionGraph.hpp>

using namespace llvm;
namespace composition {
graph_t &ProtectionGraph::getGraph() {
  return Graph;
}

vd_t ProtectionGraph::add_vertex(llvm::Value *v) {
  graph_t &g = Graph;

  auto idx = reinterpret_cast<vertex_idx_t>(v);
  if (vertexCache.find(idx) != vertexCache.end()) {
    auto vd = vertexCache.at(idx);
    g[vd].removed = false;
    return vd;
  }

  auto vd = boost::add_vertex(g);
  g[vd] = vertex_t(idx, llvmToVertexName(v), llvmToVertexType(v), {});
  vertexCache.insert({idx, vd});
  return vd;
}

void ProtectionGraph::remove_vertex(vd_t vd) noexcept {
  graph_t &g = Graph;
  g[vd].removed = true;
  for (auto[ei, ei_end] = boost::in_edges(vd, g); ei != ei_end; ++ei) {
    remove_edge(*ei);
  }
  for (auto[ei, ei_end] = boost::out_edges(vd, g); ei != ei_end; ++ei) {
    remove_edge(*ei);
  }
}

ed_t ProtectionGraph::add_edge(vd_t s, vd_t d, edge_t e) {
  graph_t &g = Graph;
  auto edge = boost::add_edge(s, d, g);
  assert(edge.second);
  g[edge.first] = std::move(e);
  return edge.first;
}

void ProtectionGraph::remove_edge(ed_t ed) noexcept {
  graph_t &g = Graph;
  g[ed].removed = true;
}

void ProtectionGraph::removeManifest(Manifest* m) {
  graph_t &g = Graph;

  for (auto[it, it_end] = Protections.left.equal_range(m); it != it_end; ++it) {
    for (auto[ei, ei_end] = boost::edges(g); ei != ei_end; ++ei) {
      auto &e = g[*ei];
      if (e.index == it->second) {
        remove_edge(*ei);
      }
    }

    for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
      auto &v = g[*vi];
      v.constraints.erase(it->second);
    }
  }
  Protections.left.erase(m);

  for (auto[it, it_end] = DependencyUndo.right.equal_range(m); it != it_end; ++it) {
    removeManifest(it->second);
  }
  DependencyUndo.right.erase(m);
  ManifestRegistry::Remove(m);
}

void ProtectionGraph::expandToInstructions() {
  // 1) Loop over all nodes
  // 2) If node is not an instruction -> Resolve instructions and add to graph
  // 3) Add edge for all edges to instructions
  graph_t &g = Graph;

  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    auto v = g[*vi];

    switch (v.type) {
    case vertex_type::FUNCTION: {
      auto func = reinterpret_cast<llvm::Function *>(v.index);
      for (auto &B : *func) {
        this->expandBasicBlockToInstructions(*vi, &B);
      }
    }
      break;
    case vertex_type::BASICBLOCK: {
      auto B = reinterpret_cast<llvm::BasicBlock *>(v.index);
      this->expandBasicBlockToInstructions(*vi, B);
    }
      break;
    case vertex_type::INSTRUCTION: break;
    case vertex_type::VALUE: break;
    case vertex_type::UNKNOWN: break;
    }
  }
}

void ProtectionGraph::expandBasicBlockToInstructions(vd_t it, llvm::BasicBlock *B) {
  graph_t &g = Graph;

  for (auto &I : *B) {
    auto node = this->add_vertex(&I);
    auto constraints = g[it].constraints;
    g[node].constraints.insert(constraints.begin(), constraints.end());
    replaceTarget(it, node);
  }
}

void ProtectionGraph::reduceToInstructions() {
  graph_t &g = Graph;

  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    auto &v = g[*vi];
    switch (v.type) {
    case vertex_type::FUNCTION:remove_vertex(*vi);
      break;
    case vertex_type::BASICBLOCK:remove_vertex(*vi);
      break;
    case vertex_type::INSTRUCTION: {
      if (boost::out_degree(*vi, g) == 0 && boost::in_degree(*vi, g) == 0) {
        remove_vertex(*vi);
      }
      break;
    }
    case vertex_type::VALUE: break;
    case vertex_type::UNKNOWN: break;
    }
  }
}

void ProtectionGraph::expandToFunctions() {
  graph_t &g = Graph;

  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    auto v = g[*vi];
    switch (v.type) {
    case vertex_type::FUNCTION: break;
    case vertex_type::BASICBLOCK: {
      const auto &B = reinterpret_cast<llvm::BasicBlock *>(v.index);
      this->expandBasicBlockToFunction(*vi, B);
    }
      break;
    case vertex_type::INSTRUCTION: {
      const auto &I = reinterpret_cast<llvm::Instruction *>(v.index);
      this->expandInstructionToFunction(*vi, I);
    }
      break;
    case vertex_type::VALUE: break;
    case vertex_type::UNKNOWN: break;
    }
  }
}

void ProtectionGraph::expandBasicBlockToFunction(vd_t it, llvm::BasicBlock *B) {
  auto func = B->getParent();
  if (func == nullptr) {
    return;
  }

  auto funcNode = this->add_vertex(func);
  replaceTarget(it, funcNode);
}

void ProtectionGraph::expandInstructionToFunction(vd_t it, llvm::Instruction *I) {
  if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
    return;
  }
  auto funcNode = this->add_vertex(I->getParent()->getParent());
  replaceTarget(it, funcNode);
}

void ProtectionGraph::replaceTarget(vd_t src, vd_t dst) {
  replaceTargetIncomingEdges(src, dst);
  replaceTargetOutgoingEdges(src, dst);
}

void ProtectionGraph::replaceTargetIncomingEdges(vd_t src, vd_t dst) {
  graph_t &g = Graph;

  for (auto[vi, vi_end] = boost::in_edges(src, g); vi != vi_end; ++vi) {
    auto e = g[*vi];
    if (e.type != edge_type::DEPENDENCY)
      continue;

    auto from = boost::source(*vi, g);
    this->add_edge(from, dst, edge_t{e.index, e.name, e.type});
  }
}

void ProtectionGraph::replaceTargetOutgoingEdges(vd_t src, vd_t dst) {
  graph_t &g = Graph;

  for (auto[vi, vi_end] = boost::out_edges(src, g); vi != vi_end; ++vi) {
    auto e = g[*vi];
    if (e.type != edge_type::DEPENDENCY)
      continue;

    auto to = boost::target(*vi, g);
    this->add_edge(dst, to, edge_t{e.index, e.name, e.type});
  }
}

void ProtectionGraph::reduceToFunctions() {
  graph_t &g = Graph;

  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    auto &v = g[*vi];
    switch (v.type) {
    case vertex_type::FUNCTION:
      if (boost::out_degree(*vi, g) == 0 && boost::in_degree(*vi, g) == 0) {
        remove_vertex(*vi);
      }
      break;
    case vertex_type::BASICBLOCK: {
      remove_vertex(*vi);
    }
      break;
    case vertex_type::INSTRUCTION: {
      remove_vertex(*vi);
    }
      break;
    case vertex_type::VALUE: break;
    case vertex_type::UNKNOWN: break;

    }
  }
}

ProtectionIndex ProtectionGraph::addConstraint(Manifest* m, std::shared_ptr<Constraint> c) {
  graph_t &g = Graph;

  if (auto d = dyn_cast<Dependency>(c.get())) {
    auto dstNode = this->add_vertex(d->getFrom());
    auto srcNode = this->add_vertex(d->getTo());
    this->add_edge(srcNode, dstNode, edge_t{ProtectionIdx, c->getInfo(), edge_type::DEPENDENCY});
  } else if (auto present = dyn_cast<Present>(c.get())) {
    auto v = this->add_vertex(present->getTarget());
    g[v].constraints.insert({ProtectionIdx, c});
  } else if (auto preserved = dyn_cast<Preserved>(c.get())) {
    auto v = this->add_vertex(preserved->getTarget());
    g[v].constraints.insert({ProtectionIdx, c});
  }
  Protections.insert({m, ProtectionIdx});
  return ProtectionIdx++;
}

std::vector<Manifest*> ProtectionGraph::topologicalSortManifests(std::vector<Manifest*> manifests) {
  auto rg = filter_removed_graph(Graph);
  auto fg = filter_dependency_graph(rg);
  auto sc = filter_selfcycle_graph(fg);

  for (auto[vi, vi_end] = boost::vertices(sc); vi != vi_end; ++vi) {
    for (auto[ei, ei_end] = boost::in_edges(*vi, sc); ei != ei_end; ++ei) {
      auto i = sc[*ei].index;
      auto manifest = Protections.right.find(i)->second;
      manifests.erase(std::remove(manifests.begin(), manifests.end(), manifest), manifests.end());
    }
  }

  auto sorted = reverse_topological_sort(sc);
  for (auto v : sorted) {
    for (auto[ei, ei_end] = boost::in_edges(v, sc); ei != ei_end; ++ei) {
      auto i = sc[*ei].index;
      auto manifest = Protections.right.find(i)->second;

      if (std::find(manifests.begin(), manifests.end(), manifest) != manifests.end()) {
        continue;
      }
      manifests.push_back(manifest);
    }
  }

  return manifests;
}

void ProtectionGraph::destroy() {
  Graph = {};
  ProtectionIdx = 0;
  Protections.clear();
  vertexCache.clear();
  DependencyUndo.clear();
}

void ProtectionGraph::computeManifestDependencies() {
  /*ManifestCoverageMap coverage{};
  for (auto &m : manifests) {
    for (auto &c : m->Coverage()) {
      auto worked = coverage.insert({m, c});
      assert(worked.second && "coverage");
    }
  }
  dbgs() << "Coverage\n";
  //print_map(coverage.left);*/

  /*ProtecteeManifestMap protection{};
  for (auto &m : manifests) {
    for (auto &p : m->GuardInstructions()) {
      auto worked = protection.insert({p, m});
      assert(worked.second && "protection");
    }
  }
  dbgs() << "Protection \n";
  //print_map(protection.left);

  ManifestDependencyMap dependency{};

  dbgs() << "Dependency\n";
  for (auto &[p, m1] : protection.left) {
    for (auto[it, it_end] = coverage.right.equal_range(p); it != it_end; ++it) {
      if (m1 != it->second) {
        dbgs() << it->second->index << " - " << m1->index << "\n";
        auto worked = dependency.insert({it->second, m1});
        assert(worked.second && "dependency");
      }
    }
  }*/

  ManifestUndoMap undo{};
  for (auto &[m, i] : Protections.left) {
    for (auto it : m->UndoValues()) {
      auto worked = undo.insert({m, it});
      assert(worked.second && "undo");
    }
  }
  std::unordered_map<Manifest*, std::unordered_set<llvm::Value *>> manifestUsers{};

  for (auto&[m, u] : undo.left) {
    for (auto it = u->user_begin(), it_end = u->user_end(); it != it_end; ++it) {
      manifestUsers[m].insert(*it);
    }
  }

  for (auto &[m, users] : manifestUsers) {
    for (auto u : users) {
      for (auto[it, it_end] = undo.right.equal_range(u); it != it_end; ++it) {
        if (m != it->second) {
          //dbgs() << it->second->index << " - " << m->index << "\n";
          DependencyUndo.insert({it->second, m});
        }
      }
    }
  }
}

}