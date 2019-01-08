#ifndef COMPOSITION_FRAMEWORK_GRAPH_EDGE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_EDGE_HPP

#include <composition/graph/constraint/constraint.hpp>
#include <llvm/Support/raw_ostream.h>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

namespace composition::graph {
// edge_idx_t type.
enum class edge_idx_t : uintptr_t;
edge_idx_t& operator++(edge_idx_t& i);
const edge_idx_t operator++(edge_idx_t& i, int);
std::ostream& operator<<(std::ostream& out, const edge_idx_t& i);
bool operator<(edge_idx_t lhs, edge_idx_t rhs);

/**
 * Defines the structure of an edge in the graph.
 */
struct edge_t {
  /**
   * Unique index of the edge
   */
  edge_idx_t index;
  /**
   * Existing constraints for this edge
   */
  std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>> constraints{};

  /**
   * Creates a new edge
   * @param index the index of the edge
   * @param name the name of the edge
   * @param type the type of the edge
   */
  explicit edge_t(edge_idx_t index = edge_idx_t(0),
                  std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>
                      constraints = {}) noexcept;

  std::ostream& operator<<(std::ostream& os) noexcept;

  bool operator==(const edge_t& rhs) noexcept;

  bool operator!=(const edge_t& rhs) noexcept;
};

} // namespace composition::graph
#endif // COMPOSITION_FRAMEWORK_GRAPH_EDGE_HPP
