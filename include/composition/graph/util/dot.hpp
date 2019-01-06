#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_DOT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_DOT_HPP

#include <boost/algorithm/string/replace.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/util/constraint_map.hpp>
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
template <typename graph_t> void graph_to_dot(graph_t& g, const std::string& filename) noexcept {
  std::ofstream f;
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f.open(filename);
  graph_to_dot(g, f);
}

/**
 * Writes the graph `g` to the ostream `out` in graphviz format
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @param out the ostream to write to
 */
template <typename graph_t> void graph_to_dot(graph_t& g, std::ostream& out) {
  auto index = index_map(g);
  auto [isPresent, isPreserved] = constraint_map(g);

  boost::dynamic_properties dp;
  dp.property("node_id", boost::make_assoc_property_map(index));
  dp.property("label", boost::get(&vertex_t::name, g));
  dp.property("removed", boost::get(&vertex_t::removed, g));
  dp.property("present", boost::make_assoc_property_map(isPresent));
  dp.property("preserved", boost::make_assoc_property_map(isPreserved));

  dp.property("removed", boost::get(&edge_t::removed, g));
  boost::write_graphviz_dp(out, g, dp);
}
} // namespace composition::graph::util

#endif // COMPOSITION_FRAMEWORK_GRAPH_UTIL_DOT_HPP
