#ifndef COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP

#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <composition/graph/vertex.hpp>
#include <composition/graph/edge.hpp>
#include <composition/graph/filter/dependency.hpp>
#include <composition/graph/filter/removed.hpp>

namespace composition {
using VertexProperties = boost::property<boost::vertex_index_t, int, vertex_t>;
using EdgeProperties = boost::property<boost::edge_index_t, int, edge_t>;
using GraphProperties = boost::property<boost::graph_name_t, std::string>;

typedef boost::adjacency_list<
    boost::listS,
    boost::listS,
    boost::bidirectionalS,
    VertexProperties,
    EdgeProperties,
    GraphProperties
> graph_t;

using vd_t = graph_t::vertex_descriptor;
using ed_t = graph_t::edge_descriptor;

template<typename property_t, typename graph_t>
auto get_vertex_property(const property_t &p,
                         const typename graph_t::vertex_descriptor vd,
                         graph_t &g) noexcept -> decltype(get(get(p, g), vd)) {
  static_assert(!std::is_const<graph_t>::value, "graph cannot be const");
  const auto map = get(p, g);
  return get(map, vd);
}

template<typename property_t, typename graph_t, typename value_t>
void set_vertex_property(const property_t &p,
                         const typename graph_t::vertex_descriptor vd,
                         const value_t &value,
                         graph_t &g) noexcept {
  const auto map = get(p, g);
  put(map, vd, value);
}

template<typename property_t, typename graph_t>
auto get_edge_property(const property_t &p,
                       const typename graph_t::edge_descriptor ed,
                       graph_t &g) noexcept -> decltype(get(get(p, g), ed)) {
  const auto map = get(p, g);
  return get(map, ed);
}

template<typename property_t, typename graph_t, typename value_t>
void set_edge_property(const property_t &p,
                       const typename graph_t::edge_descriptor ed,
                       const value_t &v,
                       graph_t &g) noexcept {
  static_assert(!std::is_const<graph_t>::value, "graph cannot be const");
  const auto map = get(p, g);
  put(map, ed, v);
}

template<typename property_t, typename graph_t, typename value_t>
bool has_vertex_with_property(const property_t &p, const value_t &v, const graph_t &g) noexcept {
  using vd = typename graph_t::vertex_descriptor;
  const auto vip = boost::vertices(g);
  return std::find_if(vip.first, vip.second, [v, p, g](const vd &d) {
    const auto map = get(p, g);
    return get(map, d) == v;
  }) != vip.second;
}

template<typename property_t, typename graph_t, typename value_t>
typename graph_t::vertex_descriptor find_first_vertex_with_property(const property_t &p,
                                                                    const value_t &v,
                                                                    const graph_t &g) {
  using vd = typename graph_t::vertex_descriptor;
  const auto vip = boost::vertices(g);
  const auto i = std::find_if(vip.first, vip.second, [v, p, g](const vd &d) {
    const auto map = get(p, g);
    return get(map, d) == v;
  });

  if (i == vip.second) {
    std::stringstream msg;
    msg << __func__ << ": could not find vertex with property value '" << v << "'";
    throw std::invalid_argument(msg.str());
  }
  return *i;
}

template<typename property_t, typename graph_t, typename value_t>
typename graph_t::vertex_descriptor find_first_vertex_with_property_map(const property_t &p,
                                                                        const value_t &v,
                                                                        const graph_t &g) {
  using vd = typename graph_t::vertex_descriptor;
  const auto vip = boost::vertices(g);
  const auto i = std::find_if(vip.first, vip.second, [v, p, g](const vd &d) {
    return get(p, d) == v;
  });

  if (i == vip.second) {
    std::stringstream msg;
    msg << __func__ << ": could not find vertex with property value '" << v << "'";
    throw std::invalid_argument(msg.str());
  }
  return *i;
}
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_GRAPH_HPP
