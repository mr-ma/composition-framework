#ifndef COMPOSITION_FRAMEWORK_GRAPH_EDGE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_EDGE_HPP

#include <utility>
#include <string>
#include <composition/graph/edge_type.hpp>

namespace composition {
typedef unsigned long edge_idx_t;

struct edge_t {
  edge_idx_t index;
  std::string name;
  edge_type type;
  bool removed{};

  explicit edge_t(
      edge_idx_t index = 0,
      std::string name = "",
      edge_type type = edge_type::UNKNOWN
  ) noexcept : index{index},
               name{std::move(name)},
               type{type} {
  };

  std::ostream &operator<<(std::ostream &os) noexcept;

  bool operator==(const edge_t &rhs) noexcept;

  bool operator!=(const edge_t &rhs) noexcept;
};

}
#endif //COMPOSITION_FRAMEWORK_GRAPH_EDGE_HPP
