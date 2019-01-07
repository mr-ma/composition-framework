#include <utility>

#include <composition/graph/edge.hpp>
#include <sstream>

namespace composition::graph {

edge_idx_t& operator++(edge_idx_t& i) {
  using T = typename std::underlying_type<edge_idx_t>::type;
  auto val = static_cast<T>(i);
  i = edge_idx_t(++val);
  return i;
}

edge_idx_t operator++(edge_idx_t& i, int) {
  edge_idx_t res(i);
  ++i;
  return res;
}

std::ostream& operator<<(std::ostream& out, const edge_idx_t& i) {
  using T = typename std::underlying_type<edge_idx_t>::type;
  out << static_cast<T>(i);
  return out;
}

bool operator<(edge_idx_t lhs, edge_idx_t rhs) {
  using T = typename std::underlying_type<edge_idx_t>::type;
  return static_cast<T>(lhs) < static_cast<T>(rhs);
}

std::ostream& operator<<(std::ostream& os, const edge_type& obj) {
  os << static_cast<std::underlying_type<edge_type>::type>(obj);
  return os;
}

std::ostream& edge_t::operator<<(std::ostream& os) noexcept {
  os << this->index << "," << this->type;
  return os;
}

bool edge_t::operator==(const edge_t& rhs) noexcept { return this->index == rhs.index; }

bool edge_t::operator!=(const edge_t& rhs) noexcept { return !(*this == rhs); }

edge_t::edge_t(edge_idx_t index, edge_type type) noexcept : index(index), type(type) {}
} // namespace composition::graph