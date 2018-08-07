#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHML_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHML_HPP

#include <fstream>
#include <ostream>
#include <boost/graph/graphml.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/graph/vertex.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/util/constraint_map.hpp>

namespace composition {
template<typename graph_t>
void save_graph_to_graphml(graph_t &g, const std::string &filename) noexcept {
  std::ofstream f;
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f.open(filename);
  save_graph_to_graphml(g, f);
}

template<typename graph_t>
void save_graph_to_graphml(graph_t &g, std::ostream &out) noexcept {
  auto index = index_map(g);
  auto [isPresent, isPreserved] = constraint_map(g);

  boost::dynamic_properties dp;
  dp.property("vertex_name", get(&vertex_t::name, g));
  dp.property("vertex_removed", get(&vertex_t::removed, g));
  dp.property("vertex_present", boost::make_assoc_property_map(isPresent));
  dp.property("vertex_preserved", boost::make_assoc_property_map(isPreserved));

  dp.property("edge_name", get(&edge_t::name, g));
  dp.property("edge_removed", get(&edge_t::removed, g));

  boost::write_graphml(out, g, boost::make_assoc_property_map(index), dp);
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_UTIL_GRAPHML_HPP
