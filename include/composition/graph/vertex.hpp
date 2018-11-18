#ifndef COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP

#include <utility>
#include <string>
#include <unordered_map>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/graph/constraint.hpp>

namespace composition::graph {
//vertex_idx_t type. TODO C++ does not enforce type safety. Potentially there are ways how type safety can be improved.
typedef unsigned long vertex_idx_t;
//ConstraintIndex type. TODO C++ does not enforce type safety. Potentially there are ways how type safety can be improved.
typedef unsigned long ConstraintIndex;

/**
 * Type of the vertex in a graph
 */
enum class vertex_type {
  UNKNOWN,
  FUNCTION,
  BASICBLOCK,
  INSTRUCTION,
  VALUE
};
std::ostream &operator<<(std::ostream &os, const vertex_type &obj);


/**
 * Describes a vertex in the graph
 */
struct vertex_t {
  /**
   * Unique index of the vertex
   */
  vertex_idx_t index;
  /**
   * Name of the vertex
   */
  std::string name;
  /**
   * Type of the vertex
   */
  vertex_type type;
  /**
   * Was the vertex removed graph?
   */
  bool removed{};
  /**
   * Existing constraints for this vertex
   */
  std::unordered_map<ConstraintIndex, std::shared_ptr<Constraint>> constraints;

  explicit vertex_t(
      vertex_idx_t index = 0,
      std::string name = "",
      vertex_type type = vertex_type::UNKNOWN,
      std::unordered_map<ConstraintIndex, std::shared_ptr<Constraint>> constraints = {}
  ) noexcept;

  std::ostream &operator<<(std::ostream &os) noexcept;

  bool operator==(const vertex_t &rhs) noexcept;

  bool operator!=(const vertex_t &rhs) noexcept;
};

/**
 * Tries to convert the given `llvm::Value` to the equivalent `vertex_type`
 * @param v the value
 * @return the `vertex_type`, UNKNOWN if it cannot be determined.
 */
vertex_type llvmToVertexType(const llvm::Value *v);

/**
 * Tries to convert the given `llvm::Value` to a string representation
 * @param v the value
 * @return the string representation, empty if it cannot be determined.
 */
std::string llvmToVertexName(const llvm::Value *v);
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP
