#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP

#include <unordered_map>
#include <boost/range/iterator_range.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <composition/graph/present.hpp>
#include <composition/graph/preserved.hpp>

namespace composition::graph::util {

/**
 * Computes the present/not present, preserved/not preserved combined constraint of each vertice
 * @tparam present_t the type of the present value
 * @tparam preserved_t the type of the preserved value
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @return a pair of maps containing the combined constraints for each vertice.
 */
template<typename present_t = size_t, typename preserved_t = size_t, typename graph_t>
std::pair<std::unordered_map<typename graph_t::vertex_descriptor, present_t>,
          std::unordered_map<typename graph_t::vertex_descriptor, preserved_t>> constraint_map(graph_t &g) {
  using vd_t = typename graph_t::vertex_descriptor;
  std::unordered_map<vd_t, present_t> isPresent;
  std::unordered_map<vd_t, preserved_t> isPreserved;

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
