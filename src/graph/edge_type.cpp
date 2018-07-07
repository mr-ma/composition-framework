#include <type_traits>
#include <composition/graph/edge_type.hpp>
namespace composition {
std::ostream &operator<<(std::ostream &os, const edge_type &obj) {
  os << static_cast<std::underlying_type<edge_type>::type>(obj);
  return os;
}
}