#ifndef COMPOSITION_FRAMEWORK_GRAPH_GRAPHVIZ_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_GRAPHVIZ_HPP

#include <string>
namespace composition {
std::string graphviz_encode(std::string s) noexcept;

std::string graphviz_decode(std::string s) noexcept;
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_GRAPHVIZ_HPP
