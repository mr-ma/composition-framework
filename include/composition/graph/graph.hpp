#ifndef COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/vertex.hpp>
#include <string>

namespace composition::graph {
/**
 * Each vertex has an index of type `vertex_idx_t` and an associated struct of type `vertex_t`.
 */
using VertexProperties = boost::property<boost::vertex_index_t, vertex_idx_t, vertex_t>;
/**
 * Each each has an index of type `edge_idx_t` and an associated struct of type `edge_t`.
 */
using EdgeProperties = boost::property<boost::edge_index_t, edge_idx_t, edge_t>;
/**
 * Each graph has a name of type `std::string`.
 */
using GraphProperties = boost::property<boost::graph_name_t, std::string>;

/**
 * Describes the protection graph in boost
 */
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProperties, EdgeProperties,
                              GraphProperties>
    graph_t;

/**
 * vd_t helper
 */
using vd_t = graph_t::vertex_descriptor;
/**
 * ed_t helper
 */
using ed_t = graph_t::edge_descriptor;

} // namespace composition::graph
#endif // COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP
