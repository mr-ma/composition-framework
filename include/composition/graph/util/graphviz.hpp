#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHVIZ_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHVIZ_HPP

#include <string>
namespace composition::graph::util {
/**
 * Encodes the given string into a graphviz compatible representation
 * @param s the string to encode
 * @return the encoded string
 */
std::string graphviz_encode(std::string s) noexcept;

/**
 * Decodes the given graphviz compatible representation into a string
 * @param s the string to decode
 * @return the decoded string
 */
std::string graphviz_decode(std::string s) noexcept;
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHVIZ_HPP
