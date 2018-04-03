#include "constraints/GraphPrinter.h"

using namespace llvm;

std::unordered_map<Vertex, uint> GraphPrinter::printNodes() {
	std::unordered_map<Vertex, uint> map;
	uint i = 0;

	boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(m_Graph); vi != vi_end; vi++) {
		Vertex it = m_Graph[*vi];
		dbgs() << std::to_string(i) << " [";

		auto label = it.name;
		if (it.name.empty()) {
			switch (it.type) {
				case BASICBLOCK:
					label = "bb";
					break;
				case INSTRUCTION:
					label = "instr";
					break;
				case FUNCTION:
					label = "func";
					break;
			}
		}

		dbgs() << "label=\"" << label << "\",";


		dbgs() << "];\n";
		map[it] = i;
		i++;
	}

	return map;
}

void GraphPrinter::printEdges(std::unordered_map<Vertex, uint> map) {
	dbgs() << "{\n";
	dbgs() << "edge [color=black];\n";

	boost::graph_traits<Graph>::edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::edges(m_Graph); vi != vi_end; vi++) {
		Edge it = m_Graph[*vi];
		if (it.type != HIERARCHY) continue;

		auto from = m_Graph[boost::source(*vi, m_Graph)];
		auto to = m_Graph[boost::target(*vi, m_Graph)];

		auto fromID = map[from];
		auto toID = map[to];

		dbgs() << std::to_string(fromID) << " -> " << std::to_string(toID) << ";\n";
	}

	dbgs() << "}\n";
	dbgs() << "{\n";
	dbgs() << "edge [color=red];\n";

	for (std::tie(vi, vi_end) = boost::edges(m_Graph); vi != vi_end; vi++) {
		Edge it = m_Graph[*vi];
		if (it.type != PROTECTION) continue;
		auto from = m_Graph[boost::source(*vi, m_Graph)];
		auto to = m_Graph[boost::target(*vi, m_Graph)];

		auto fromID = map[from];
		auto toID = map[to];

		dbgs() << std::to_string(fromID) << " -> " << std::to_string(toID) << " [label=\"";
		dbgs() << "#" << std::to_string(it.protectionID) << " " << it.name << "\"]\n";
	}
	dbgs() << "}\n";
}

void GraphPrinter::dump_dot() {
	dbgs() << "digraph G {\n"
	          "concentrate=true;\n"
	          "overlap = scale;\n"
	          "splines=true;\n"
	          "node [shape=box]\n";
	auto mapping = this->printNodes();
	this->printEdges(mapping);
	dbgs() << "}\n";
}
