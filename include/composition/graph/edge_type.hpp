#ifndef COMPOSITION_FRAMEWORK_GRAPH_EDGETYPE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_EDGETYPE_HPP

#include <llvm/Support/raw_ostream.h>
namespace composition {
enum class edge_type {
  UNKNOWN,
  CFG,
  DEPENDENCY,
};

std::ostream &operator<<(std::ostream &os, const edge_type &obj);
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_EDGETYPE_HPP
