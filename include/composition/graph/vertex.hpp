#ifndef COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP

#include <utility>
#include <string>
#include <unordered_map>
#include <llvm/IR/Value.h>
#include <composition/graph/constraint.hpp>
#include <composition/graph/vertex_type.hpp>

namespace composition {
typedef unsigned long vertex_idx_t;
typedef unsigned long ConstraintIndex;

struct vertex_t {
  vertex_idx_t index;
  std::string name;
  vertex_type type;
  std::unordered_map<ConstraintIndex, std::shared_ptr<Constraint>> constraints;
  bool removed{};

  explicit vertex_t(
      vertex_idx_t index = 0,
      std::string name = "",
      vertex_type type = vertex_type::UNKNOWN,
      std::unordered_map<ConstraintIndex, std::shared_ptr<Constraint>> constraints = {}
  ) noexcept;;

  std::ostream &operator<<(std::ostream &os) noexcept;

  bool operator==(const vertex_t &rhs) noexcept;

  bool operator!=(const vertex_t &rhs) noexcept;
};

void assertType(llvm::Value *value, vertex_type type);

vertex_type llvmToVertexType(const llvm::Value *v);

std::string llvmToVertexName(const llvm::Value *v);
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP
