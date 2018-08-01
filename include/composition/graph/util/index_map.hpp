#ifndef COMPOSITION_FRAMEWORK_INDEX_MAP_HPP
#define COMPOSITION_FRAMEWORK_INDEX_MAP_HPP

#include <cstddef>
#include <map>
#include <boost/graph/adjacency_list.hpp>

template<typename s_t = size_t, typename graph_t>
std::map<typename graph_t::vertex_descriptor, s_t> index_map(graph_t &g) {
  std::map<typename graph_t::vertex_descriptor, s_t> map;
  s_t idx = 0;
  for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
    map.insert({*vi, idx++});
  }
  return map;
}

#endif //COMPOSITION_FRAMEWORK_INDEX_MAP_HPP
