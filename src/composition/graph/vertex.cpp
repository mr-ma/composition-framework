#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/graph/vertex.hpp>
#include <composition/graph/util/graphviz.hpp>
#include <composition/util/strings.hpp>
namespace composition {

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

vertex_t::vertex_t(vertex_idx_t index,
                   std::string name,
                   vertex_type type,
                   std::unordered_map<ConstraintIndex, std::shared_ptr<Constraint>> constraints) noexcept :
    index{index},
    name{std::move(name)},
    type{type},
    constraints{std::move(constraints)} {
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

vertex_type llvmToVertexType(const llvm::Value *v) {
  assert(v != nullptr && "Value for llvmToVertexType is nullptr");

  if (llvm::isa<llvm::Instruction>(v)) {
    return vertex_type::INSTRUCTION;
  }
  if (llvm::isa<llvm::BasicBlock>(v)) {
    return vertex_type::BASICBLOCK;
  }
  if (llvm::isa<llvm::Function>(v)) {
    return vertex_type::FUNCTION;
  }
  return vertex_type::VALUE;
}

std::string llvmToVertexName(const llvm::Value *v) {
  std::string name{};
  if (auto *I = llvm::dyn_cast<llvm::Instruction>(v)) {
    if (I->getParent() != nullptr && I->getParent()->getParent() != nullptr) {
      name =
          I->getFunction()->getName().str() + "_" + std::to_string(reinterpret_cast<uintptr_t>(I->getFunction())) + "_";
    }
    name += std::to_string(reinterpret_cast<uintptr_t>(v));
  } else if (auto *F = llvm::dyn_cast<llvm::Function>(v)) {
    name = F->getName().str() + "_" + std::to_string(reinterpret_cast<uintptr_t>(F));
  } else {
    name = v->getName();
  }
  ltrim(name);
  return name;
}
}