#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_FILTER_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_FILTER_HPP
#include <boost/graph/filtered_graph.hpp>

namespace composition::graph::filter { 

/*
 * Filters provide a zerocopy representation of a graph. Vertices and edges are filtered on the fly while iterating.
 * For expensive filters or large graph creating a copy of the graph can provide performance improvements.
 * This could be achieved with boost::copy_graph.
 *
 * TODO The definition below does not support perfect forwarding as I could not figure out how we can handle different return types.
 * Thus, stacking filters into each other is not working.
 * Example (Wanted):
 * graph g;
 * auto f = filter1(filter2(g));
 *
 * Example (What is working):
 * graph g;
 * auto f2 = filter2(g);
 * auto f1 = filter1(f2);
 *
 *
 */
/**
 * Template definition of a filtered graph for custom filters
 * @tparam filter the type of the filter
 * @tparam graph_t the type of the graph `g`
 * @param g the graph to filter
 * @return a filtered graph representation of `g`
 */
template<template<typename> typename filter, typename graph_t>
auto filter_graph(graph_t &g) -> decltype(boost::make_filtered_graph(g, filter<graph_t>(g), filter<graph_t>(g))) {
  return boost::make_filtered_graph(g, filter<graph_t>(g), filter<graph_t>(g));
}

}

#endif //COMPOSITION_FRAMEWORK_GRAPH_FILTER_FILTER_HPP
