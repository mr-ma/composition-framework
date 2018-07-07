#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <composition/graph/vertex.hpp>
#include <composition/graph/graphviz.hpp>
namespace composition {

vertex_t::vertex_t(
    vertex_idx_t index,
    const std::string &name,
    vertex_type type,
    const std::unordered_map<ConstraintIndex, std::shared_ptr<Constraint>> &constraints
) noexcept :
    index{index},
    name{name},
    type{type},
    constraints{constraints} {
}

std::ostream &vertex_t::operator<<(std::ostream &os) noexcept {
  os << this->index
     << ","
     << graphviz_encode(this->name)
     << ","
     << this->type
     << ",";
  for (const auto &c : this->constraints) {
    os << c.second->getInfo() << " ";
  }
  return os;
}

bool vertex_t::operator==(const vertex_t &rhs) noexcept {
  return this->index == rhs.index;
}

bool vertex_t::operator!=(const vertex_t &rhs) noexcept {
  return !(*this == rhs);
}

void assertType(llvm::Value *value, vertex_type type) {
  assert(value != nullptr && "Value for assertType is nullptr");

  if (llvm::isa<llvm::Instruction>(value)) {
    assert(type == vertex_type::INSTRUCTION);
  } else if (llvm::isa<llvm::BasicBlock>(value)) {
    assert(type == vertex_type::BASICBLOCK);
  } else if (llvm::isa<llvm::Function>(value)) {
    assert(type == vertex_type::FUNCTION);
  } else if (llvm::isa<llvm::Value>(value)) {
    assert(type == vertex_type::VALUE);
  } else {
    assert(false);
  }
}

vertex_type llvmToVertexType(const llvm::Value *value) {
  assert(value != nullptr && "Value for llvmToVertexType is nullptr");

  if (llvm::isa<llvm::Instruction>(value)) {
    return vertex_type::INSTRUCTION;
  }
  if (llvm::isa<llvm::BasicBlock>(value)) {
    return vertex_type::BASICBLOCK;
  }
  if (llvm::isa<llvm::Function>(value)) {
    return vertex_type::FUNCTION;
  }
  return vertex_type::VALUE;
}

}