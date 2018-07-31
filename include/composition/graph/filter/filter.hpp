#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_FILTER_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_FILTER_HPP
#include <boost/graph/filtered_graph.hpp>

namespace composition {
template<template<typename> typename filter, typename graph_t>
auto filter_graph(graph_t &g) -> decltype(boost::make_filtered_graph(g, filter<graph_t>(g), filter<graph_t>(g))) {
  return boost::make_filtered_graph(g, filter<graph_t>(g), filter<graph_t>(g));
}

}

#endif //COMPOSITION_FRAMEWORK_GRAPH_FILTER_FILTER_HPP
