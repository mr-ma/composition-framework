#ifndef COMPOSITION_FRAMEWORK_FILTERED_GRAPH_HIDDEN_HPP
#define COMPOSITION_FRAMEWORK_FILTERED_GRAPH_HIDDEN_HPP

#include <boost/graph/filtered_graph.hpp>
#include <composition/graph/graph.hpp>

namespace composition {
template<typename T>
struct HiddenPredicate { // both edge and vertex
  bool operator()(typename T::edge_descriptor ed) const {
    assert(G != nullptr);
    return !(*G)[ed].removed;
  }

  bool operator()(typename T::vertex_descriptor vd) const {
    assert(G != nullptr);
    return !(*G)[vd].removed;
  }

  T *G;

  HiddenPredicate() = default;

  explicit HiddenPredicate(T &G) : G(&G) {}
};

typedef boost::filtered_graph<graph_t, HiddenPredicate<graph_t>> graph_hidden_t;
}

#endif //COMPOSITION_FRAMEWORK_FILTERED_GRAPH_HIDDEN_HPP
