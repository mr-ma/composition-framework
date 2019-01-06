#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
#include <boost/graph/filtered_graph.hpp>
#include <composition/graph/edge.hpp>

namespace composition::graph::filter {
/**
 * Predicate to filter reflexive edges in a graph
 * @tparam T the type of the graph
 */
template <typename T> struct SelfCyclePredicate {
private:
  T* g;

public:
  bool operator()(typename T::edge_descriptor ed) const { return boost::source(ed, *g) != boost::target(ed, *g); }

  bool operator()(typename T::vertex_descriptor vd) const { return true; }

  SelfCyclePredicate() : g(nullptr) {}

  explicit SelfCyclePredicate(T& g) : g(&g) { assert(&g != nullptr); }
};

/**
 * Filters all edges which are reflexive
 * @tparam graph_t the type of the graph `g`
 * @param g the graph to filter
 * @return a filtered representation of `g` which hides all reflexive edges
 */
template <typename graph_t>
auto filter_selfcycle_graph(graph_t& g)
    -> decltype(boost::make_filtered_graph(g, SelfCyclePredicate<graph_t>(g), SelfCyclePredicate<graph_t>(g))) {
  return boost::make_filtered_graph(g, SelfCyclePredicate<graph_t>(g), SelfCyclePredicate<graph_t>(g));
}
} // namespace composition::graph::filter
#endif // COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
