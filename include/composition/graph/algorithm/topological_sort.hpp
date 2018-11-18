#ifndef COMPOSITION_FRAMEWORK_GRAPH_ALGORITHM_TOPOLOGICALSORT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_ALGORITHM_TOPOLOGICALSORT_HPP

#include <vector>
#include <map>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <composition/graph/util/index_map.hpp>

namespace composition::graph::algorithm {

/**
 * Algorithm to compute a topological sort of the graph `g`
 * @tparam graph_t the type of `g` to support any type of graph
 * @param g the graph to calculate the topological sort
 * @return a vector of vertices in topological order
 */
template<typename graph_t>
std::vector<typename graph_t::vertex_descriptor> topological_sort(graph_t &g) {
  auto index = index_map(g);
  auto assocIndexMap = boost::make_assoc_property_map(index);

  std::vector<typename graph_t::vertex_descriptor> result;
  boost::topological_sort(g, std::back_inserter(result), vertex_index_map(assocIndexMap));

  return result;
}

/**
 * Algorithm to compute a reverse topological sort of the graph `g`
 * @tparam graph_t the type of `g` to support any type of graph
 * @param g the graph to calculate the topological sort
 * @return a vector of vertices in reversed topological order
 */
template<typename graph_t>
std::vector<typename graph_t::vertex_descriptor> reverse_topological_sort(graph_t &g) {
  //TODO this can probably be replaced by a copy of topological_sort and replacing std::back_inserter with front inserter, thus saving on O(vertices) time and space.
  auto result = topological_sort(g);
  std::reverse(result.begin(), result.end());
  return result;
}
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_ALGORITHM_TOPOLOGICALSORT_HPP
