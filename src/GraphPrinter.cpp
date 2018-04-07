#include "constraints/GraphPrinter.h"

using namespace llvm;

std::unordered_map<Vertex, uint> GraphPrinter::printNodes() {
	std::unordered_map<Vertex, uint> map;
	uint i = 0;

	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		Vertex it = Graph[*vi];
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

	boost::graph_traits<graph_t>::edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::edges(Graph); vi != vi_end; vi++) {
		Edge it = Graph[*vi];
		if (it.type != HIERARCHY) continue;

		auto from = Graph[boost::source(*vi, Graph)];
		auto to = Graph[boost::target(*vi, Graph)];

		auto fromID = map[from];
		auto toID = map[to];

		dbgs() << std::to_string(fromID) << " -> " << std::to_string(toID) << ";\n";
	}

	dbgs() << "}\n";
	dbgs() << "{\n";
	dbgs() << "edge [color=red];\n";

	for (std::tie(vi, vi_end) = boost::edges(Graph); vi != vi_end; vi++) {
		Edge it = Graph[*vi];
		if (it.type != PROTECTION) continue;
		auto from = Graph[boost::source(*vi, Graph)];
		auto to = Graph[boost::target(*vi, Graph)];

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
