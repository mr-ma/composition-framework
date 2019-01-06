#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
#include <boost/graph/filtered_graph.hpp>
#include <composition/graph/edge.hpp>

namespace composition::graph::filter {
/**
 * Predicate to filter all edges which are not a dependency edge.
 * @tparam T the type of the graph
 */
template <typename T> struct DependencyPredicate {
private:
  T* g;

public:
  bool operator()(typename T::edge_descriptor ed) const { return (*g)[ed].type == edge_type::DEPENDENCY; }

  bool operator()(typename T::vertex_descriptor vd) const { return true; }

  DependencyPredicate() : g(nullptr) {}

  explicit DependencyPredicate(T& g) : g(&g) { assert(&g != nullptr); }
};

/**
 * Filters all edges which are not a dependency edge
 * @tparam graph_t the type of the graph `g`
 * @param g the graph to filter
 * @return a filtered representation of `g` which hides all edges which are not a dependency edge
 */
template <typename graph_t>
auto filter_dependency_graph(graph_t& g)
    -> decltype(boost::make_filtered_graph(g, DependencyPredicate<graph_t>(g), DependencyPredicate<graph_t>(g))) {
  return boost::make_filtered_graph(g, DependencyPredicate<graph_t>(g), DependencyPredicate<graph_t>(g));
}
} // namespace composition::graph::filter
#endif // COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
