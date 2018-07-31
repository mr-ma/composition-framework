#ifndef COMPOSITION_FRAMEWORK_VERTEX_COUNT_HPP
#define COMPOSITION_FRAMEWORK_VERTEX_COUNT_HPP

#include <cstddef>
#include <boost/graph/adjacency_list.hpp>

template<typename graph_t>
size_t vertex_count(graph_t &g) {
  size_t vertexCount = 0;
  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    ++vertexCount;
  }
  return vertexCount;
}

#endif //COMPOSITION_FRAMEWORK_VERTEX_COUNT_HPP
