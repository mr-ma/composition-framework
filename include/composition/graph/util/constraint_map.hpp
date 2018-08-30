#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP

#include <map>
#include <boost/range/iterator_range.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <composition/graph/constraint.hpp>

namespace composition {

template<typename present_t = size_t, typename preserved_t = size_t, typename graph_t>
std::pair<std::map<typename graph_t::vertex_descriptor, present_t>,
          std::map<typename graph_t::vertex_descriptor, preserved_t>> constraint_map(graph_t &g) {
  using vd_t = typename graph_t::vertex_descriptor;
  std::map<vd_t, present_t> isPresent;
  std::map<vd_t, preserved_t> isPreserved;

  for (auto vd :boost::make_iterator_range(boost::vertices(g))) {
    PresentConstraint present = PresentConstraint::NONE;
    PreservedConstraint preserved = PreservedConstraint::NONE;
    for (auto &c : g[vd].constraints) {
      if (auto *p1 = llvm::dyn_cast<Preserved>(c.second.get())) {
        preserved = p1->isInverse() ?
                    preserved | PreservedConstraint::NOT_PRESERVED :
                    preserved | PreservedConstraint::PRESERVED;
      } else if (auto *p2 = llvm::dyn_cast<Present>(c.second.get())) {
        present = p2->isInverse() ?
                  present | PresentConstraint::NOT_PRESENT :
                  present | PresentConstraint::PRESENT;
      }
    }
    isPresent.insert({vd, static_cast<present_t>(present)});
    isPreserved.insert({vd, static_cast<preserved_t>(preserved)});
  }

  return {isPresent, isPreserved};
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP
