#include <cstdint>
#include <unordered_set>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/graph/ProtectionGraph.hpp>

using namespace llvm;
namespace composition {
vd_t ProtectionGraph::insertNode(llvm::Value *input, vertex_type type) {
  assertType(input, type);

  auto idx = reinterpret_cast<uintptr_t>(input);
  if(vertices.find(idx) != vertices.end()) {
    return vertices[idx];
  }

  if (has_vertex_with_property(&vertex_t::index, idx, GraphWithHidden)) {
    return find_first_vertex_with_property(&vertex_t::index, idx, GraphWithHidden);
  }

  std::string name;
  if (!input->hasName()) {
    if (isa<Instruction>(input)) {
      name = "I";
    } else if (isa<BasicBlock>(input)) {
      name = "BB";
    }
  } else {
    name = input->getName();
  }

  auto vd = boost::add_vertex(vertex_t(idx, name, type, {}), GraphWithHidden);
  vertices.insert({idx, vd});
  return vd;
}

void ProtectionGraph::removeProtection(ProtectionIndex protectionID) {
  Protections.erase(protectionID);

  auto [vi, vi_end] = boost::edges(Graph);
  for (auto next = vi; next != vi_end; vi = next) {
    ++next;

    auto & e = Graph[*vi];
    if (e.type == edge_type::DEPENDENCY) {
      if (e.index == protectionID) {
        e.removed = true;
      }
    }
  }
}

void ProtectionGraph::expandToInstructions() {
  // 1) Loop over all nodes
  // 2) If node is not an instruction -> Resolve instructions and add to graph
  // 3) Add edge for all edges to instructions
  for (auto [vi, vi_end] = boost::vertices(Graph); vi != vi_end; ++vi) {
    auto v = Graph[*vi];

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
  for (auto &I : *B) {
    auto node = this->insertNode(&I, vertex_type::INSTRUCTION);
    replaceTarget(it, node);
  }
}

void ProtectionGraph::reduceToInstructions() {
  auto [vi, vi_end] = boost::vertices(GraphWithHidden);
  for (auto next = vi; vi != vi_end; vi = next) {
    ++next;
    auto &v = Graph[*vi];
    switch (v.type) {
    case vertex_type::FUNCTION:
      boost::clear_vertex(*vi, GraphWithHidden);
      v.removed = true;
      break;
    case vertex_type::BASICBLOCK:
      boost::clear_vertex(*vi, GraphWithHidden);
      v.removed = true;
      break;
    case vertex_type::INSTRUCTION: {
      if (boost::out_degree(*vi, GraphWithHidden) == 0 && boost::in_degree(*vi, GraphWithHidden) == 0) {
        v.removed = true;
      }
      break;
    }
    case vertex_type::VALUE: break;
    case vertex_type::UNKNOWN: break;
    }
  }
}

graph_hidden_t &ProtectionGraph::getGraph() {
  return Graph;
}
graph_t &ProtectionGraph::getGraphWithHidden() {
  return GraphWithHidden;
}

void ProtectionGraph::expandToFunctions() {
  for (auto [vi, vi_end] = boost::vertices(Graph); vi != vi_end; ++vi) {
    auto v = Graph[*vi];
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
  if (B->getParent() == nullptr) {
    return;
  }
  auto func = B->getParent();
  assert(func != nullptr);
  auto funcNode = this->insertNode(func, vertex_type::FUNCTION);

  replaceTarget(it, funcNode);
}

void ProtectionGraph::expandInstructionToFunction(vd_t it, llvm::Instruction *I) {
  if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
    return;
  }
  auto func = I->getParent()->getParent();
  assert(func != nullptr);
  auto funcNode = this->insertNode(func, vertex_type::FUNCTION);

  replaceTarget(it, funcNode);
}

void ProtectionGraph::replaceTarget(vd_t src, vd_t dst) {
  replaceTargetIncomingEdges(src, dst);
  replaceTargetOutgoingEdges(src, dst);
}

void ProtectionGraph::replaceTargetIncomingEdges(vd_t src, vd_t dst) {
  for (auto [vi, vi_end] = boost::in_edges(src, Graph); vi != vi_end; ++vi) {
    auto e = Graph[*vi];
    if (e.type != edge_type::DEPENDENCY)
      continue;

    auto from = boost::source(*vi, Graph);
    auto edge = boost::add_edge(from, dst, GraphWithHidden);
    assert(edge.second);

    edge_t eNew(e.index, e.name, e.type);
    Graph[edge.first] = eNew;
  }
}

void ProtectionGraph::replaceTargetOutgoingEdges(vd_t src, vd_t dst) {
  for (auto [vi, vi_end] = boost::out_edges(src, Graph); vi != vi_end; ++vi) {
    auto e = Graph[*vi];
    if (e.type != edge_type::DEPENDENCY)
      continue;

    auto to = boost::target(*vi, Graph);
    auto edge = boost::add_edge(dst, to, GraphWithHidden);
    assert(edge.second);

    edge_t eNew(e.index, e.name, e.type);
    Graph[edge.first] = eNew;
  }
}

void ProtectionGraph::reduceToFunctions() {
  auto [vi, vi_end] = boost::vertices(Graph);
  for (auto next = vi; vi != vi_end; vi = next) {
    ++next;

    auto& v = Graph[*vi];
    switch (v.type) {
    case vertex_type::FUNCTION:
      if (boost::out_degree(*vi, Graph) == 0 && boost::in_degree(*vi, Graph) == 0) {
        v.removed = true;
      }
      break;
    case vertex_type::BASICBLOCK:
      boost::clear_vertex(*vi, GraphWithHidden);
      v.removed = true;
      break;
    case vertex_type::INSTRUCTION: {
      boost::clear_vertex(*vi, GraphWithHidden);
      v.removed = true;
    }
      break;
    case vertex_type::VALUE: break;
    case vertex_type::UNKNOWN: break;

    }
  }
}

void ProtectionGraph::handleCycle(std::vector<vd_t> matches) {
  dbgs() << "Handling cycle in component\n";

  vd_t prev;
  for (auto it = matches.begin(), it_end = matches.end(); it != it_end; ++it) {
    vd_t vd = *it;
    auto v = Graph[vd];

    dbgs() << v.name << "\n";
    dbgs() << std::to_string(v.index) << "\n";

    if (it == matches.begin()) {
      prev = vd;
      continue;
    }

    boost::remove_edge(vd, prev, GraphWithHidden);
    boost::remove_edge(prev, vd, GraphWithHidden);

    prev = vd;
  }
}

ProtectionIndex ProtectionGraph::addConstraint(ManifestIndex index, std::shared_ptr<Constraint> c) {
  if (auto d = dyn_cast<Dependency>(c.get())) {
    assert(d->getFrom() != nullptr && "Source edge is nullptr");
    assert(d->getTo() != nullptr && "Target edge is nullptr");

    auto dstNode = this->insertNode(d->getFrom(), llvmToVertexType(d->getFrom()));
    auto srcNode = this->insertNode(d->getTo(), llvmToVertexType(d->getTo()));

    auto edge = boost::add_edge(srcNode, dstNode, GraphWithHidden);
    assert(edge.second);
    Graph[edge.first] = edge_t{ProtectionIdx, c->getInfo(), edge_type::DEPENDENCY};
  } else if (auto present = dyn_cast<Present>(c.get())) {
    assert(present->getTarget() != nullptr);
    auto v = this->insertNode(present->getTarget(), llvmToVertexType(present->getTarget()));
    Graph[v].constraints.insert({ProtectionIdx, c});
  } else if (auto preserved = dyn_cast<Preserved>(c.get())) {
    assert(preserved->getTarget() != nullptr);
    auto v = this->insertNode(preserved->getTarget(), llvmToVertexType(preserved->getTarget()));
    Graph[v].constraints.insert({ProtectionIdx, c});
  }
  Protections[ProtectionIdx] = index;
  return ProtectionIdx++;
}
}