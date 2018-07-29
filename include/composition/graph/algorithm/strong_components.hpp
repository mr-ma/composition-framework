#ifndef COMPOSITION_FRAMEWORK_GRAPH_STRONGCOMPONENTS_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_STRONGCOMPONENTS_HPP

#include <boost/graph/strong_components.hpp>
#include <boost/graph/detail/adjacency_list.hpp>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>

namespace composition {

template<typename graph_t>
std::vector<std::vector<typename graph_t::vertex_descriptor>> SCC(graph_t &g) {
  using vd_t = typename graph_t::vertex_descriptor;
  using ed_t = typename graph_t::edge_descriptor;
  using ei_t = typename graph_t::edge_iterator;

  auto index = index_map(g);
  auto assocIndexMap = boost::make_assoc_property_map(index);

  auto vertices = boost::num_vertices(g);
  std::vector<int> component(vertices), discover_time(vertices);
  std::vector<boost::default_color_type> color(vertices);
  std::vector<vd_t> root(vertices);
  auto num =
      static_cast<unsigned long>(boost::strong_components(g, boost::iterator_property_map(component.begin(), assocIndexMap),
                                                          root_map(boost::make_iterator_property_map(root.begin(),
                                                                                                     assocIndexMap)).
                                                              discover_time_map(boost::make_iterator_property_map(
                                                              discover_time.begin(),
                                                              assocIndexMap)).color_map(
                                                              make_iterator_property_map(color.begin(), assocIndexMap))
      ));

  llvm::dbgs() << "Total number of components: " << std::to_string(num) << "\n";
  std::vector<std::vector<vd_t>> result;
  result.reserve(num);
  for (auto i = 0; i != num; i++) {
    std::vector<vd_t> matches;
    for (auto &el : index) {
      if (component[el.second] == i) {
        llvm::dbgs() << "Vertex " << std::to_string(el.second) << " " << g[el.first].name << " is in component "
                     << std::to_string(component[el.second])
                     << "\n";
        matches.emplace_back(el.first);
      }
    }

    result.emplace_back(matches);
  }

  return result;
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_STRONGCOMPONENTS_HPP
