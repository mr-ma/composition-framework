#include <composition/GraphPrinter.hpp>
#include <composition/graph/vertex_type.hpp>
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

		auto v = Graph[*vi];
		auto idx = v.index;
		auto name = v.name;
		auto type = v.type;

		if (type == vertex_type::INSTRUCTION) {
			auto &&instr = reinterpret_cast<Instruction *>(idx);
			auto funcName = instr->getParent()->getParent()->getName().str();
			clusterMap.insert({funcName, *vi});
			continue;
		}

		auto label = name;
		if (name.empty()) {
			switch (type) {
				case vertex_type::BASICBLOCK: {
					auto &&b = reinterpret_cast<BasicBlock *>(idx);
					label = b->getParent()->getName().str() + "_bb_" + std::to_string(idx);
				}
					break;
				case vertex_type::FUNCTION:
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
		auto v = Graph[it->second];
		auto label = "i_" + std::to_string(v.index);
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
		auto e = Graph[*vi];
		auto type = e.type;

		if (type != edge_type::CFG) continue;

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
		auto e = Graph[*vi];
		auto idx = e.index;
		auto type = e.type;
		auto name = e.name;

		if (type != edge_type::DEPENDENCY) continue;
		auto from = boost::source(*vi, Graph);
		auto fromID = map[from];
		auto fromName = get_vertex_property(&vertex_t::name, from, Graph);

		auto to = boost::target(*vi, Graph);
		auto toID = map[to];
		auto toName = get_vertex_property(&vertex_t::name, to, Graph);

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