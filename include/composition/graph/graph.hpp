#ifndef COMPOSITION_FRAMEWORK_GRAPH_H
#define COMPOSITION_FRAMEWORK_GRAPH_H

#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <composition/graph/vertex_type.hpp>
#include <composition/graph/edge_type.hpp>
#include <composition/graph/vertex_attribute_set.hpp>

namespace composition {
	typedef uintptr_t vertex_idx_t;
	typedef uintptr_t edge_idx_t;

	typedef boost::adjacency_list<
			boost::listS,
			boost::listS,
			boost::bidirectionalS,
			boost::property<boost::vertex_index_t, vertex_idx_t,
					boost::property<boost::vertex_name_t, std::string,
							boost::property<boost::vertex_type_t, vertex_type,
									boost::property<boost::vertex_attribute_set_t, vertex_attribute_set>
							>
					>
			>,
			boost::property<boost::edge_index_t, edge_idx_t,
					boost::property<boost::edge_name_t, std::string,
							boost::property<boost::edge_type_t, edge_type>
					>
			>,
			boost::property<boost::graph_name_t, std::string>
	> graph_t;

	using vd = graph_t::vertex_descriptor;
	using ed = graph_t::edge_descriptor;

	template<typename property_t, typename graph_t>
	auto get_vertex_property(const property_t &p, const typename graph_t::vertex_descriptor vd, graph_t &g) noexcept -> decltype(get(get(p, g), vd)) {
		static_assert(!std::is_const<graph_t>::value, "graph cannot be const");
		const auto map = get(p, g);
		return get(map, vd);
	}

	template<typename property_t, typename graph_t, typename value_t>
	void set_vertex_property(const property_t &p, const typename graph_t::vertex_descriptor vd, const value_t &value, graph_t &g) noexcept {
		const auto map = get(p, g);
		put(map, vd, value);
	}

	template<typename property_t, typename graph_t>
	auto get_edge_property(const property_t &p, const typename graph_t::edge_descriptor ed, graph_t &g) noexcept -> decltype(get(get(p, g), ed)) {
		const auto map = get(p, g);
		return get(map, ed);
	}

	template<typename property_t, typename graph_t, typename value_t>
	void set_edge_property(const property_t &p, const typename graph_t::edge_descriptor ed, const value_t &v, graph_t &g) noexcept {
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
	typename graph_t::vertex_descriptor find_first_vertex_with_property(const property_t &p, const value_t &v, const graph_t &g) {
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
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_H
