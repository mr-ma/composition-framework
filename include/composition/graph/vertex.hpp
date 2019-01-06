#ifndef COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP

#include <composition/graph/constraint/constraint.hpp>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>
#include <string>
#include <unordered_map>
#include <utility>

namespace composition::graph {
// vertex_idx_t type.
enum vertex_idx_t : uintptr_t;
vertex_idx_t& operator++(vertex_idx_t& i);
vertex_idx_t operator++(vertex_idx_t& i, int);

/**
 * Type of the vertex in a graph
 */
enum class vertex_type { UNKNOWN, FUNCTION, BASICBLOCK, INSTRUCTION, VALUE };
std::ostream& operator<<(std::ostream& os, const vertex_type& obj);

/**
 * Describes a vertex in the graph
 */
struct vertex_t {
  /**
   * Unique index of the vertex
   */
  vertex_idx_t index;
  /**
   * LLVM value of the vertex
   */
  llvm::Value* value;
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
  std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>> constraints;

  uint64_t absoluteHotness = 0;
  float hotness = 0;
  float coverage = 0;

  /**
   * Creates a new vertex
   * @param index the index of the vertex
   * @param name the name of the vertex
   * @param type the type of the vertex
   * @param constraints the constraints of the vertex
   */
  explicit vertex_t(vertex_idx_t index = vertex_idx_t(0), llvm::Value* value = nullptr, std::string name = "",
                    vertex_type type = vertex_type::UNKNOWN,
                    std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>
                        constraints = {}) noexcept;

  std::ostream& operator<<(std::ostream& os) noexcept;

  bool operator==(const vertex_t& rhs) noexcept;

  bool operator!=(const vertex_t& rhs) noexcept;
};

/**
 * Tries to convert the given `llvm::Value` to the equivalent `vertex_type`
 * @param v the value
 * @return the `vertex_type`, UNKNOWN if it cannot be determined.
 */
vertex_type llvmToVertexType(const llvm::Value* v);

/**
 * Tries to convert the given `llvm::Value` to a string representation
 * @param v the value
 * @return the string representation, empty if it cannot be determined.
 */
std::string llvmToVertexName(const llvm::Value* v);
} // namespace composition::graph

#endif // COMPOSITION_FRAMEWORK_GRAPH_VERTEX_HPP
