#ifndef COMPOSITION_FRAMEWORK_DOT_HPP
#define COMPOSITION_FRAMEWORK_DOT_HPP

#include <fstream>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <composition/graph/empty_graph.hpp>
#include <boost/algorithm/string/replace.hpp>
#include "graphviz.hpp"

namespace composition {
	template<typename graph>
	class bundled_vertices_writer {
	public:
		explicit bundled_vertices_writer(graph g) : g{g} {}

		template<class vertex_descriptor>
		void operator()(std::ostream &out, const vertex_descriptor &vd) const noexcept {
			out << "[label=" << g[vd].name << "]";
		}

	private:
		graph g;
	};

	template<typename graph>
	class bundled_edges_writer {
	public:
		explicit bundled_edges_writer(graph g) : g{g} {}

		template<class edge_descriptor>
		void operator()(std::ostream &out, const edge_descriptor &ed) const noexcept {
			out << "[label=" << g[ed].name << "]";
		}

	private:
		graph g;
	};

	template<typename graph>
	inline bundled_vertices_writer<graph>
	make_bundled_vertices_writer(const graph &g) {
		return bundled_vertices_writer<graph>(g);
	}

	template<typename graph>
	inline bundled_edges_writer<graph>
	make_bundled_edges_writer(const graph &g) {
		return bundled_edges_writer<graph>(g);
	}

	template<typename graph>
	void save_graph_to_dot(graph &g, const std::string &filename) noexcept {
		std::map<vd, size_t> index;
		auto pmap = boost::make_assoc_property_map(index);

		for (auto vd : boost::make_iterator_range(boost::vertices(g))) {
			index[vd] = index.size();
		}

		std::ofstream f(filename);
		boost::write_graphviz(f, g, make_bundled_vertices_writer(g), make_bundled_edges_writer(g), boost::default_writer(), pmap);
	}

	bool is_regular_file(const std::string &filename) noexcept {
		std::fstream f;
		f.open(filename.c_str(), std::ios::in);
		return f.is_open();
	}

	graph_t load_graph_from_dot(const std::string &dot_filename) {
		if (!is_regular_file(dot_filename)) {
			std::stringstream msg;
			msg << __func__ << " : ␣ f i l e ␣ ’ " << dot_filename << " ’ ␣not ␣ found ";
			throw std::invalid_argument(msg.str());
		}

		std::ifstream f(dot_filename.c_str());
		auto g = create_empty_graph();
		boost::dynamic_properties dp(
				boost::ignore_other_properties
		);
		boost::read_graphviz(f, g, dp);
		return g;
	}
}

#endif //COMPOSITION_FRAMEWORK_DOT_HPP
