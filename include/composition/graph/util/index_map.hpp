#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_INDEXMAP_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_INDEXMAP_HPP

#include <cstddef>
#include <unordered_map>
#include <boost/graph/adjacency_list.hpp>

namespace composition::graph::util {
/**
 * Associates an index with each vertice
 * @tparam s_t type of the index
 * @tparam graph_t type of the graph `g`
 * @param g the graph
 * @return a mapping from vertex descriptors to indices.
 */
template<typename s_t = size_t, typename graph_t>
std::unordered_map<typename graph_t::vertex_descriptor, s_t> index_map(graph_t &g) {
  std::unordered_map<typename graph_t::vertex_descriptor, s_t> map;
  s_t idx = 0;
  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    map.insert({*vi, idx++});
  }
  return map;
}
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_UTIL_INDEXMAP_HPP
