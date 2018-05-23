#include <composition/GraphPrinter.hpp>
#include "MMap.h"

using namespace llvm;
using namespace composition;

std::unordered_map<vd, uint> GraphPrinter::printNodes(raw_ostream &stream) {
	std::unordered_map<vd, uint> idxMap;
	MMap<std::string, vd, MMapComp<std::pair<std::string, vd>>> clusterMap;
	uint i = 0;

	graph_t::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		idxMap[*vi] = i;
		i++;

		auto idx = get_vertex_property(boost::vertex_index, *vi, Graph);
		auto name = get_vertex_property(boost::vertex_name, *vi, Graph);
		auto type = get_vertex_property(boost::vertex_type, *vi, Graph);

		if (type == INSTRUCTION) {
			auto &&instr = reinterpret_cast<Instruction *>(idx);
			auto funcName = instr->getParent()->getParent()->getName().str();
			clusterMap.insert({funcName, *vi});
			continue;
		}

		auto label = name;
		if (name.empty()) {
			switch (type) {
				case BASICBLOCK: {
					auto &&b = reinterpret_cast<BasicBlock *>(idx);
					label = b->getParent()->getName().str() + "_bb_" + std::to_string(idx);
				}
					break;
				case FUNCTION:
					label = "func";
					break;
				default:
					break;
			}
		}

		stream << std::to_string(idxMap[*vi]) << " [label=\"" << label << "\"];\n";
	}

	std::string last;
	for (auto it = std::begin(clusterMap), it_end = std::end(clusterMap); it != it_end; it++) {
		if (last.empty()) {
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
		auto label = "i_" + std::to_string(get_vertex_property(boost::vertex_index, it->second, Graph));
		stream << std::to_string(idxMap[it->second]) << " [label=\"" << label << "\"];\n";
	}
	if (!last.empty()) {
		stream << "};\n";
	}

	return idxMap;
}

void GraphPrinter::printEdges(std::unordered_map<vd, uint> map, raw_ostream &stream) {
	stream << "{\n";
	stream << "edge [color=black];\n";

	graph_t::edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::edges(Graph); vi != vi_end; vi++) {
		auto type = get_edge_property(boost::edge_type, *vi, Graph);

		if (type != HIERARCHY) continue;

		auto from = boost::source(*vi, Graph);
		auto to = boost::target(*vi, Graph);

		auto fromID = map[from];
		auto toID = map[to];

		stream << std::to_string(fromID) << " -> " << std::to_string(toID) << ";\n";
	}

	stream << "}\n";
	stream << "{\n";
	stream << "edge [color=red];\n";

	for (std::tie(vi, vi_end) = boost::edges(Graph); vi != vi_end; vi++) {
		auto idx = get_edge_property(boost::edge_index, *vi, Graph);
		auto type = get_edge_property(boost::edge_type, *vi, Graph);
		auto name = get_edge_property(boost::edge_name, *vi, Graph);

		if (type != PROTECTION) continue;
		auto from = boost::source(*vi, Graph);
		auto fromID = map[from];
		auto fromName = get_vertex_property(boost::vertex_name, from, Graph);

		auto to = boost::target(*vi, Graph);
		auto toID = map[to];
		auto toName = get_vertex_property(boost::vertex_name, to, Graph);

		dbgs() << fromName << " -> " << toName << "\n";
		dbgs() << std::to_string(fromID) << " -> " << std::to_string(toID) << "\n";

		stream << std::to_string(fromID) << " -> " << std::to_string(toID);
		stream << " [label=\"" << "#" << std::to_string(idx) << " " << name << "\"];\n";
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