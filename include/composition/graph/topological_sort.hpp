#ifndef COMPOSITION_FRAMEWORK_TOPOLOGICAL_SORT_HPP
#define COMPOSITION_FRAMEWORK_TOPOLOGICAL_SORT_HPP

#include <vector>
#include <map>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range_core.hpp>

namespace composition {
	template<typename g_t>
	std::vector<typename g_t::vertex_descriptor> topological_sort(g_t &g) {
		using vd_t = typename g_t::vertex_descriptor;

		std::map<vd_t, size_t> index;
		auto pmap = boost::make_assoc_property_map(index);

		const auto &it = boost::make_iterator_range(boost::vertices(g));
		int idx = 0;
		for (auto vd : it) {
			put(pmap, vd, idx++);
		}
		std::vector<boost::default_color_type> color(boost::num_vertices(g));

		std::vector<vd_t> result;
		boost::topological_sort(g, std::back_inserter(result), color_map(make_iterator_property_map(color.begin(), pmap)));

		return result;
	}

	template<typename g_t>
	std::vector<typename g_t::vertex_descriptor> reverse_topological_sort(g_t &g) {
		auto result = topological_sort(g);
		std::reverse(result.begin(), result.end());
		return result;
	}
}

#endif //COMPOSITION_FRAMEWORK_TOPOLOGICAL_SORT_HPP
