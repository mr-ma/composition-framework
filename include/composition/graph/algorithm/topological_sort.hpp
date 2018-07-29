#ifndef COMPOSITION_FRAMEWORK_GRAPH_TOPOLOGICALSORT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_TOPOLOGICALSORT_HPP

#include <vector>
#include <map>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <composition/graph/graph.hpp>

namespace composition {
template<typename g_t>
std::vector<typename g_t::vertex_descriptor> topological_sort(g_t &g) {
  using vd_t = typename g_t::vertex_descriptor;

  auto index = index_map(g);
  auto assocIndexMap = boost::make_assoc_property_map(index);

  std::vector<boost::default_color_type> color(boost::num_vertices(g));

  std::vector<vd_t> result;
  boost::topological_sort(g, std::back_inserter(result), color_map(make_iterator_property_map(color.begin(), assocIndexMap)));

  return result;
}

template<typename g_t>
std::vector<typename g_t::vertex_descriptor> reverse_topological_sort(g_t &g) {
  auto result = topological_sort(g);
  std::reverse(result.begin(), result.end());
  return result;
}
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_TOPOLOGICALSORT_HPP
