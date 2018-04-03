#include "constraints/GraphPrinter.h"

using namespace llvm;

std::unordered_map<Node, uint> GraphPrinter::printNodes() {
	std::unordered_map<Node, uint> map;
	uint i = 0;
	for (auto it = m_Graph.begin(); it != m_Graph.end(); it++) {
		dbgs() << std::to_string(i) << " [";

		auto label = it->name;
		if (label.empty()) {
			switch (it->type) {
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
		map[*it] = i;
		i++;
	}

	return map;
}

void GraphPrinter::printEdges(std::unordered_map<Node, uint> map) {
	dbgs() << "{\n";
	dbgs() << "edge [color=black];\n";
	for (auto it = m_Graph.arc_begin(); it != m_Graph.arc_end(); it++) {
		if (it->type != HIERARCHY) continue;
		auto from = m_Graph.arc_from(it);
		auto to = m_Graph.arc_to(it);

		auto fromID = map[*from];
		auto toID = map[*to];

		dbgs() << std::to_string(fromID) << " -> " << std::to_string(toID) << ";\n";
	}

	dbgs() << "}\n";
	dbgs() << "{\n";
	dbgs() << "edge [color=red];\n";
	for (auto it = m_Graph.arc_begin(); it != m_Graph.arc_end(); it++) {
		if (it->type != PROTECTION) continue;
		auto from = m_Graph.arc_from(it);
		auto to = m_Graph.arc_to(it);

		auto fromID = map[*from];
		auto toID = map[*to];

		dbgs() << std::to_string(fromID) << " -> " << std::to_string(toID) << " [label=\"";
		dbgs() << "#" << std::to_string(it->protectionID) << " " << it->name << "\"]\n";
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
