#include <type_traits>
#include <composition/graph/vertex_type.hpp>
namespace composition {
std::ostream &operator<<(std::ostream &os, const vertex_type &obj) {
  os << static_cast<std::underlying_type<vertex_type>::type>(obj);
  return os;
}
}