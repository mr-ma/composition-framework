#ifndef COMPOSITION_FRAMEWORK_FILTERED_GRAPH_HIDDEN_HPP
#define COMPOSITION_FRAMEWORK_FILTERED_GRAPH_HIDDEN_HPP

namespace composition {
template<typename T>
struct RemovedPredicate { // both edge and vertex
  bool operator()(typename T::edge_descriptor ed) const {
    assert(G != nullptr);
    return !(*G)[ed].removed;
  }

  bool operator()(typename T::vertex_descriptor vd) const {
    assert(G != nullptr);
    return !(*G)[vd].removed;
  }

  T *G;

  RemovedPredicate() = default;

  explicit RemovedPredicate(T &G) : G(&G) {}
};
}
#endif //COMPOSITION_FRAMEWORK_FILTERED_GRAPH_HIDDEN_HPP
