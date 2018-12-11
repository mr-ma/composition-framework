#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
#include <composition/graph/edge.hpp>
#include <composition/graph/filter/filter.hpp>

namespace composition::graph::filter {
/**
 * Predicate to filter reflexive edges in a graph
 * @tparam T the type of the graph
 */
template <typename T> struct SelfCyclePredicate {
  bool operator()(typename T::edge_descriptor ed) const {
    assert(g != nullptr);
    auto source = boost::source(ed, *g);
    auto target = boost::target(ed, *g);
    return source != target;
  }

  bool operator()(typename T::vertex_descriptor vd) const {
    assert(g != nullptr);
    return true;
  }

  T* g;

  SelfCyclePredicate() : g(nullptr) {}

  explicit SelfCyclePredicate(T& g) : g(&g) {}
};

/**
 * Filters all edges which are reflexive
 * @tparam graph_t the type of the graph `g`
 * @param g the graph to filter
 * @return a filtered representation of `g` which hides all reflexive edges
 */
template <typename graph_t> auto filter_selfcycle_graph(graph_t& g) -> decltype(filter_graph<SelfCyclePredicate>(g)) {
  return filter_graph<SelfCyclePredicate>(g);
}
} // namespace composition::graph::filter
#endif // COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
