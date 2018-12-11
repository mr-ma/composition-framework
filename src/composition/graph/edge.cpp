#include <utility>

#include <composition/graph/edge.hpp>
#include <composition/graph/util/graphviz.hpp>
#include <sstream>

namespace composition::graph {
using util::graphviz_encode;

std::ostream& operator<<(std::ostream& os, const edge_type& obj) {
  os << static_cast<std::underlying_type<edge_type>::type>(obj);
  return os;
}

std::ostream& edge_t::operator<<(std::ostream& os) noexcept {
  os << this->index << "," << graphviz_encode(this->name) << "," << this->type;
  return os;
}

bool edge_t::operator==(const edge_t& rhs) noexcept { return this->index == rhs.index; }

bool edge_t::operator!=(const edge_t& rhs) noexcept { return !(*this == rhs); }

edge_t::edge_t(edge_idx_t index, std::string name, edge_type type) noexcept
    : index(index), name(std::move(name)), type(type) {}
} // namespace composition::graph