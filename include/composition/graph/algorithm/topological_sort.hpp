#ifndef COMPOSITION_FRAMEWORK_GRAPH_TOPOLOGICALSORT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_TOPOLOGICALSORT_HPP

#include <vector>
#include <map>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <composition/graph/util/index_map.hpp>

namespace composition {
template<typename graph_t>
std::vector<typename graph_t::vertex_descriptor> reverse_topological_sort(graph_t &g) {
  using vd_t = typename graph_t::vertex_descriptor;

  auto index = index_map(g);
  auto assocIndexMap = boost::make_assoc_property_map(index);

  std::vector<boost::default_color_type> color(boost::num_vertices(g));

  std::vector<vd_t> result;
  boost::topological_sort(g, std::back_inserter(result), color_map(make_iterator_property_map(color.begin(), assocIndexMap)));

  return result;
}

template<typename graph_t>
std::vector<typename graph_t::vertex_descriptor> topological_sort(graph_t &g) {
  auto result = reverse_topological_sort(g);
  std::reverse(result.begin(), result.end());
  return result;
}
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_TOPOLOGICALSORT_HPP
