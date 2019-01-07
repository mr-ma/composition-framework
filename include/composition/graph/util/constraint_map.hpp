#ifndef COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/range/iterator_range.hpp>
#include <composition/graph/constraint/present.hpp>
#include <composition/graph/constraint/preserved.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/vertex.hpp>
#include <unordered_map>

namespace composition::graph::util {

using composition::graph::constraint::Present;
using composition::graph::constraint::PresentConstraint;
using composition::graph::constraint::Preserved;
using composition::graph::constraint::PreservedConstraint;
/**
 * Computes the present/not present, preserved/not preserved combined constraint of each vertice
 * @tparam present_t the type of the present value
 * @tparam preserved_t the type of the preserved value
 * @tparam graph_t the type of the graph `g`
 * @param g the graph
 * @return a pair of maps containing the combined constraints for each vertice.
 */
/*
template <typename present_t = size_t, typename preserved_t = size_t, typename graph_t>
std::pair<std::unordered_map<vertex_idx_t, present_t>, std::unordered_map<vertex_idx_t, preserved_t>>
constraint_map(graph_t& g, const std::map<vertex_idx_t, vertex_t>& VERTICES,
               const boost::bimaps::bimap<vertex_idx_t, typename graph_t::vertex_descriptor>& VERTICES_DESCRIPTORS) {
  using vd_t = typename graph_t::vertex_descriptor;

  std::unordered_map<vertex_idx_t, present_t> isPresent;
  std::unordered_map<vertex_idx_t, preserved_t> isPreserved;

  for (const vd_t& vd : boost::make_iterator_range(boost::vertices(g))) {
    vertex_idx_t idx = VERTICES_DESCRIPTORS.right.at(vd);
    const vertex_t& v = VERTICES.at(idx);

    PresentConstraint present = PresentConstraint::NONE;
    PreservedConstraint preserved = PreservedConstraint::NONE;
    for (auto& c : v.constraints) {
      if (auto* p1 = llvm::dyn_cast<Preserved>(c.second.get())) {
        preserved = p1->isInverse() ? preserved | PreservedConstraint::NOT_PRESERVED
                                    : preserved | PreservedConstraint::PRESERVED;
      } else if (auto* p2 = llvm::dyn_cast<Present>(c.second.get())) {
        present = p2->isInverse() ? present | PresentConstraint::NOT_PRESENT : present | PresentConstraint::PRESENT;
      }
    }
    isPresent.insert({idx, static_cast<present_t>(present)});
    isPreserved.insert({idx, static_cast<preserved_t>(preserved)});
  }

  return {isPresent, isPreserved};
}*/
} // namespace composition::graph::util
#endif // COMPOSITION_FRAMEWORK_GRAPH_UTIL_CONSTRAINTMAP_HPP
