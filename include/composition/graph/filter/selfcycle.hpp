#ifndef COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
#include <composition/graph/edge_type.hpp>
#include <composition/graph/filter/filter.hpp>

namespace composition {
template<typename T>
struct SelfCyclePredicate { // both edge and vertex
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

  T *g;

  SelfCyclePredicate() : g(nullptr) {}

  explicit SelfCyclePredicate(T &g) : g(&g) {}
};

template<typename graph_t>
auto filter_selfcycle_graph(graph_t &g) -> decltype(filter_graph<SelfCyclePredicate>(g)) {
  return filter_graph<SelfCyclePredicate>(g);
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_FILTER_SELFCYCLE_HPP
