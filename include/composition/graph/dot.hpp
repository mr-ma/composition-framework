#ifndef COMPOSITION_FRAMEWORK_DOT_HPP
#define COMPOSITION_FRAMEWORK_DOT_HPP

#include <fstream>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <composition/graph/empty_graph.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace composition {
	std::string graphviz_encode(std::string s) noexcept {
		boost::algorithm::replace_all(s, " , ", "$$$COMMA$$$");
		boost::algorithm::replace_all(s, " ", "$$$SPACE$$$");
		boost::algorithm::replace_all(s, "\"", "$$$QUOTE$$$");
		return s;
	}

	std::string grapviz_decode(std::string s) noexcept {
		boost::algorithm::replace_all(s, "$$$COMMA$$$", " , ");
		boost::algorithm::replace_all(s, "$$$SPACE$$$", " ");
		boost::algorithm::replace_all(s, "$$$QUOTE$$$", "\"");
		return s;
	}

	class custom_vertex_writer {
	public:
		explicit custom_vertex_writer(graph_t &g) : g(g) {}

		template<class Vertex>
		void operator()(std::ostream &out, const Vertex &e) const {
			out << "[label=" << get_vertex_property(boost::vertex_name, e, g) << "]";
		}

	private:
		graph_t &g;
	};

	class custom_edge_writer {
	public:
		explicit custom_edge_writer(graph_t &g) : g(g) {}

		template<class Edge>
		void operator()(std::ostream &out, const Edge &e) const {
			out << "[label=" << get_edge_property(boost::edge_name, e, g) << "]";
		}

	private:
		graph_t &g;
	};

	void save_graph_to_dot(graph_t &g, const std::string &filename) noexcept {
		std::ofstream f(filename);
		boost::write_graphviz(f, g, custom_vertex_writer(g), custom_edge_writer(g));
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
