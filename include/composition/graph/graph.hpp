#ifndef COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP

#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <composition/graph/vertex.hpp>
#include <composition/graph/edge.hpp>

namespace composition {
using VertexProperties = boost::property<boost::vertex_index_t, int, vertex_t>;
using EdgeProperties = boost::property<boost::edge_index_t, int, edge_t>;
using GraphProperties = boost::property<boost::graph_name_t, std::string>;

typedef boost::adjacency_list<
    boost::vecS,
    boost::vecS,
    boost::bidirectionalS,
    VertexProperties,
    EdgeProperties,
    GraphProperties
> graph_t;

using vd_t = graph_t::vertex_descriptor;
using ed_t = graph_t::edge_descriptor;

}
#endif //COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP
