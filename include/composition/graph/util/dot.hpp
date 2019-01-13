#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_DOT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_DOT_HPP

#include <boost/algorithm/string/replace.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/vertex.hpp>
#include <fstream>
#include <ostream>

namespace composition::graph::util {
/**
 * Writes the graph `g` to the file `filename` in graphviz format
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @param filename the file to write to
 */
template <typename graph_t>
void graph_to_dot(graph_t& g, const std::string& filename, const std::unordered_map<vertex_idx_t, size_t>& isPresent,
                  const std::unordered_map<vertex_idx_t, size_t>& isPreserved) {
  std::ofstream f;
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f.open(filename);
  graph_to_dot(g, f, isPresent, isPreserved);
}

/**
 * Writes the graph `g` to the ostream `out` in graphviz format
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @param out the ostream to write to
 */
template <typename graph_t>
void graph_to_dot(graph_t& g, std::ostream& out, const std::unordered_map<vertex_idx_t, size_t>& isPresent,
                  const std::unordered_map<vertex_idx_t, size_t>& isPreserved) {}
} // namespace composition::graph::util

#endif // COMPOSITION_FRAMEWORK_GRAPH_UTIL_DOT_HPP
