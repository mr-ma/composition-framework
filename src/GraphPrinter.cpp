#include "constraints/GraphPrinter.hpp"
#include "MMap.h"

using namespace llvm;

std::unordered_map<Vertex, uint> GraphPrinter::printNodes(raw_ostream &stream) {
	std::unordered_map<Vertex, uint> idxMap;
	MMap<std::string, Vertex, MMapComp<std::pair<std::string, Vertex>>> clusterMap;
	uint i = 0;

	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		Vertex it = Graph[*vi];

		idxMap[it] = i;
		i++;

		if (it.type == INSTRUCTION) {
			auto &&instr = reinterpret_cast<Instruction *>(it.ID);
			auto funcName = instr->getParent()->getParent()->getName().str();
			clusterMap.insert({funcName, it});
			continue;
		}

		auto label = it.name;
		if (it.name.empty()) {
			switch (it.type) {
				case BASICBLOCK: {
					auto &&b = reinterpret_cast<BasicBlock *>(it.ID);
					label = b->getParent()->getName().str() + "_bb_" + std::to_string(it.ID);
				}
					break;
				case FUNCTION:
					label = "func";
					break;
				default:
					break;
			}
		}

		stream << std::to_string(i) << " [label=\"" << label << "\"];\n";
	}

	std::string last;
	for(auto it = std::begin(clusterMap), it_end = std::end(clusterMap); it != it_end; it++ ) {
		if(last.empty()) {
			stream << "subgraph cluster_" + it->first << " {\n";
			stream << "node [style=filled];\n";
			stream << "label = \"" << it->first << "\";\n";
			last = it->first;
		} else if (last != it->first) {
			stream << "};\n";
			stream << "subgraph cluster_" + it->first << " {\n";
			stream << "node [style=filled];\n";
			stream << "label = \"" << it->first << "\";\n";
			last = it->first;
		}
		auto label = "i_" + std::to_string(it->second.ID);
		stream << std::to_string(idxMap[it->second]) << " [label=\"" << label << "\"];\n";
	}
	if(!last.empty()) {
		stream << "};\n";
	}

	return idxMap;
}

void GraphPrinter::printEdges(std::unordered_map<Vertex, uint> map, raw_ostream &stream) {
	stream << "{\n";
	stream << "edge [color=black];\n";

	boost::graph_traits<graph_t>::edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::edges(Graph); vi != vi_end; vi++) {
		Edge it = Graph[*vi];
		if (it.type != HIERARCHY) continue;

		auto from = Graph[boost::source(*vi, Graph)];
		auto to = Graph[boost::target(*vi, Graph)];

		auto fromID = map[from];
		auto toID = map[to];

		stream << std::to_string(fromID) << " -> " << std::to_string(toID) << ";\n";
	}

	stream << "}\n";
	stream << "{\n";
	stream << "edge [color=red];\n";

	for (std::tie(vi, vi_end) = boost::edges(Graph); vi != vi_end; vi++) {
		Edge it = Graph[*vi];
		if (it.type != PROTECTION) continue;
		auto from = Graph[boost::source(*vi, Graph)];
		auto to = Graph[boost::target(*vi, Graph)];

		auto fromID = map[from];
		auto toID = map[to];

		stream << std::to_string(fromID) << " -> " << std::to_string(toID) << ";\n";
		//stream << " [label=\"" << "#" << std::to_string(it.protectionID) << " " << it.name << "\"]\n";
	}
	stream << "}\n";
}

void GraphPrinter::dump_dot(raw_ostream &stream) {
	stream << "digraph G {\n"
	          "concentrate=true;\n"
	          "overlap = scale;\n"
	          "splines=true;\n"
	          "node [shape=box]\n";
	auto mapping = this->printNodes(stream);
	this->printEdges(mapping, stream);
	stream << "}\n";
}

void GraphPrinter::dump_dot(std::string fileName) {
	std::error_code ec;
	auto &&output = raw_fd_ostream(fileName, ec, llvm::sys::fs::OpenFlags::F_RW);
	if (ec) {
		output.close();
		return;
	}

	dump_dot(output);
	output.close();
}