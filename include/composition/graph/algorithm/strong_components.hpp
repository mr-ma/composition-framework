#ifndef COMPOSITION_FRAMEWORK_GRAPH_ALGORITHM_STRONGCOMPONENTS_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_ALGORITHM_STRONGCOMPONENTS_HPP

#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/strong_components.hpp>
#include <composition/graph/util/index_map.hpp>
#include <vector>

namespace composition::graph::algorithm {

/**
 * Algorithm to compute the strong components in a graph.
 * @tparam graph_t the type of `g` to support any type of graph
 * @param g the graph to calculate the strong components
 * @return a vector of filtered graphs where each element is a strong component
 */
template <typename graph_t>
std::vector<boost::filtered_graph<graph_t, std::function<bool(typename graph_t::edge_descriptor)>,
                                  std::function<bool(typename graph_t::vertex_descriptor)>>>
strong_components(graph_t& g) {
  using vd_t = typename graph_t::vertex_descriptor;
  using ed_t = typename graph_t::edge_descriptor;

  auto index = std::make_shared<decltype(index_map(g))>(index_map(g));
  auto assocIndexMap = boost::make_assoc_property_map(*index);

  auto mapping = std::make_shared<std::vector<unsigned long>>(boost::num_vertices(g));
  size_t num = boost::strong_components(g, boost::iterator_property_map(mapping->begin(), assocIndexMap),
                                        vertex_index_map(assocIndexMap));

  // idea src# https://stackoverflow.com/a/26778230
  std::vector<boost::filtered_graph<graph_t, std::function<bool(ed_t)>, std::function<bool(vd_t)>>> component_graphs;

  for (size_t i = 0; i < num; ++i) {
    component_graphs.push_back({g,
                                [mapping, i, index, &g](ed_t ed) {
                                  return mapping->at(index->find(boost::source(ed, g))->second) == i ||
                                         mapping->at(index->find(boost::target(ed, g))->second) == i;
                                },
                                [mapping, i, index](vd_t vd) { return mapping->at(index->find(vd)->second) == i; }});
  }

  return component_graphs;
}
} // namespace composition::graph::algorithm
#endif // COMPOSITION_FRAMEWORK_GRAPH_ALGORITHM_STRONGCOMPONENTS_HPP
