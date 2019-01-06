#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_REMOVED_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_REMOVED_HPP
#include <boost/graph/filtered_graph.hpp>

namespace composition::graph::filter {
/**
 * Predicate to filter all vertices and edges which are marked as removed
 * @tparam T the type of the graph
 */
template <typename T> struct RemovedPredicate {
private:
  T* g;

public:
  bool operator()(typename T::edge_descriptor ed) const { return !(*g)[ed].removed; }

  bool operator()(typename T::vertex_descriptor vd) const { return !(*g)[vd].removed; }

  RemovedPredicate() = default;

  explicit RemovedPredicate(T& g) : g(&g) { assert(&g != nullptr); }
};

/**
 * Filters all vertices and edges which are marked as removed
 * @tparam graph_t the type of the graph `g`
 * @param g the graph to filter
 * @return a filtered representation of `g` which hides all removed vertices and edges
 */
template <typename graph_t>
auto filter_removed_graph(graph_t& g)
    -> decltype(boost::make_filtered_graph(g, RemovedPredicate<graph_t>(g), RemovedPredicate<graph_t>(g))) {
  return boost::make_filtered_graph(g, RemovedPredicate<graph_t>(g), RemovedPredicate<graph_t>(g));
}
} // namespace composition::graph::filter
#endif // COMPOSITION_FRAMEWORK_GRAPH_FILTER_REMOVED_HPP
