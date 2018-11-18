#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
#include <composition/graph/edge.hpp>
#include <composition/graph/filter/filter.hpp>

namespace composition::graph::filter {
/**
 * Predicate to filter all edges which are not a dependency edge.
 * @tparam T the type of the graph
 */
template<typename T>
struct DependencyPredicate {
  bool operator()(typename T::edge_descriptor ed) const {
    assert(g != nullptr);
    return (*g)[ed].type == edge_type::DEPENDENCY;
  }

  bool operator()(typename T::vertex_descriptor vd) const {
    assert(g != nullptr);
    return true;
  }

  T *g;

  DependencyPredicate() : g(nullptr) {}

  explicit DependencyPredicate(T &g) : g(&g) {}
};

/**
 * Filters all edges which are not a dependency edge
 * @tparam graph_t the type of the graph `g`
 * @param g the graph to filter
 * @return a filtered representation of `g` which hides all edges which are not a dependency edge
 */
template<typename graph_t>
auto filter_dependency_graph(graph_t &g) -> decltype(filter_graph<DependencyPredicate>(g)) {
  return filter_graph<DependencyPredicate>(g);
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
