#ifndef COMPOSITION_FRAMEWORK_GRAPH_VERTEXTYPE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_VERTEXTYPE_HPP

#include <llvm/Support/raw_ostream.h>
namespace composition {

enum class vertex_type {
  UNKNOWN,
  FUNCTION,
  BASICBLOCK,
  INSTRUCTION,
  VALUE
};
std::ostream &operator<<(std::ostream &os, const vertex_type &obj);

}
#endif //COMPOSITION_FRAMEWORK_GRAPH_VERTEXTYPE_HPP
