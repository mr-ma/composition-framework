#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
#include <composition/graph/edge_type.hpp>
#include <composition/graph/filter/filter.hpp>

namespace composition {
template<typename T>
struct DependencyPredicate { // both edge and vertex
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

template<typename graph_t>
auto filter_dependency_graph(graph_t &g) -> decltype(filter_graph<DependencyPredicate>(g)) {
  return filter_graph<DependencyPredicate>(g);
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_FILTER_DEPENDENCY_HPP
