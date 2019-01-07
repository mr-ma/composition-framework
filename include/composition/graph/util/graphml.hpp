#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHML_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHML_HPP

#include <boost/algorithm/string/replace.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/util/constraint_map.hpp>
#include <composition/graph/util/index_map.hpp>
#include <composition/graph/vertex.hpp>
#include <fstream>
#include <ostream>

namespace composition::graph::util {
using composition::graph::util::index_map;
/**
 * Writes the graph `g` to the file `filename` in graphml format
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @param filename the file to write to
 */
template <typename graph_t>
void graph_to_graphml(graph_t& g, const std::string& filename,
                      const std::unordered_map<vertex_idx_t, size_t>& isPresent,
                      const std::unordered_map<vertex_idx_t, size_t>& isPreserved) {
  std::ofstream f;
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f.open(filename);
  graph_to_graphml(g, f, isPresent, isPreserved);
}

/**
 * Writes the graph `g` to the ostream `out` in graphml format
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @param out the ostream to write to
 */
template <typename graph_t>
void graph_to_graphml(graph_t& g, std::ostream& out, const std::unordered_map<vertex_idx_t, size_t>& isPresent,
                      const std::unordered_map<vertex_idx_t, size_t>& isPreserved) {
  // auto index = std::move(index_map(g));

  // boost::dynamic_properties dp;
  //  dp.property("vertex_present", boost::make_assoc_property_map(isPresent));
  // dp.property("vertex_preserved", boost::make_assoc_property_map(isPreserved));

  // boost::write_graphml(out, g, boost::make_assoc_property_map(index), dp);
}
} // namespace composition::graph::util
#endif // COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHML_HPP
