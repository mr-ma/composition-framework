#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_VERTEXCOUNT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_VERTEXCOUNT_HPP

#include <boost/graph/adjacency_list.hpp>
#include <cstddef>

namespace composition::graph::util {
/**
 * Counts the vertices in the graph
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @return the number of vertices in the graph
 */
template <typename graph_t> size_t vertex_count(graph_t& g) {
  size_t vertexCount = 0;
  for (auto [vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    ++vertexCount;
  }
  return vertexCount;
}
} // namespace composition::graph::util
#endif // COMPOSITION_FRAMEWORK_GRAPH_UTIL_VERTEXCOUNT_HPP
