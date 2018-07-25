#include <composition/graph/edge.hpp>

#include <sstream>
#include <composition/graph/graphviz.hpp>

namespace composition {

std::ostream &edge_t::operator<<(std::ostream &os) noexcept {
  os << this->index
     << ","
     << graphviz_encode(this->name)
     << ","
     << this->type;
  return os;
}

bool edge_t::operator==(const edge_t &rhs) noexcept {
  return this->index == rhs.index;
}

bool edge_t::operator!=(const edge_t &rhs) noexcept {
  return !(*this == rhs);
}
}