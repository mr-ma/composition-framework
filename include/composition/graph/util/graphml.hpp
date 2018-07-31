#ifndef COMPOSITION_FRAMEWORK_GML_HPP
#define COMPOSITION_FRAMEWORK_GML_HPP

#include <fstream>
#include <ostream>
#include <boost/graph/graphml.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/graph/vertex.hpp>
#include <composition/graph/edge.hpp>

namespace composition {
template<typename graph>
void save_graph_to_graphml(graph &g, const std::string &filename) noexcept {
  std::ofstream f;
  f.exceptions(std::ios::failbit | std::ios::badbit);
  f.open(filename);
  save_graph_to_graphml(g, f);
}

template<typename graph>
void save_graph_to_graphml(graph &g, std::ostream &out) noexcept {
  std::map<typename graph::vertex_descriptor, size_t> index;
  std::map<typename graph::vertex_descriptor, int> isPresent;
  std::map<typename graph::vertex_descriptor, int> isPreserved;

  for (auto vd : boost::make_iterator_range(boost::vertices(g))) {
    index[vd] = index.size();

    int present = 0;
    int preserved = 0;
    for (auto &c : g[vd].constraints) {
      if (auto *p1 = llvm::dyn_cast<Preserved>(c.second.get())) {
        //None = 0, Preserved = 1, Not Preserved = 2, Both = 3
        preserved = ((preserved == 0 || preserved == 1) && !p1->isInverse() ? 1 :
                     ((preserved == 0 || preserved == 2) && p1->isInverse() ? 2 : 3));
      } else if (auto *p2 = llvm::dyn_cast<Present>(c.second.get())) {
        //None = 0, Present = 1, Not Present = 2, Both = 3
        present = ((present == 0 || present == 1) && !p2->isInverse() ? 1 :
                   ((present == 0 || present == 2) && p2->isInverse() ? 2 : 3));
      }
    }
    isPresent[vd] = present;
    isPreserved[vd] = preserved;
  }
  boost::dynamic_properties dp;
  dp.property("vertex_name", get(&vertex_t::name, g));
  dp.property("vertex_removed", get(&vertex_t::removed, g));
  dp.property("vertex_present", boost::make_assoc_property_map(isPresent));
  dp.property("vertex_preserved", boost::make_assoc_property_map(isPreserved));

  dp.property("edge_name", get(&edge_t::name, g));
  dp.property("edge_removed", get(&edge_t::removed, g));

  boost::write_graphml(out, g, boost::make_assoc_property_map(index), dp);
}
}
#endif //COMPOSITION_FRAMEWORK_GML_HPP
