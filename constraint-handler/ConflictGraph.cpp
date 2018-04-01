#include "ConflictGraph.h"

using namespace llvm;

std::unordered_map<Node, uint> ConflictGraph::printNodes() {
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

void ConflictGraph::printEdges(std::unordered_map<Node, uint> map) {
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

void ConflictGraph::dump_dot() {
	dbgs() << "digraph G {\n"
	          "concentrate=true;\n"
	          "overlap = scale;\n"
	          "splines=true;\n"
	          "node [shape=box]";
	auto mapping = this->printNodes();
	this->printEdges(mapping);
	dbgs() << "}\n";
}


stlplus::digraph<Node, Edge>::iterator ConflictGraph::insertNode(llvm::Value *input, NodeType type) {
	auto id = reinterpret_cast<uintptr_t>(input);

	auto result = m_Nodes.find(id);
	if (result != m_Nodes.end()) {
		return result->second;
	}

	m_Nodes[id] = m_Graph.insert(Node{id, input->getName().str(), type});
	return m_Nodes[id];
}

void ConflictGraph::removeProtection(uintptr_t protectionID) {
	m_Protections.erase(protectionID);

	for (auto it = m_Graph.arc_begin(); it != m_Graph.arc_end();) {
		if (it->type == PROTECTION) {
			if (it->protectionID == protectionID) {
				it = m_Graph.arc_erase(it);
				continue;
			}
		}

		it++;
	}
}

void ConflictGraph::expand() {
	// 1) Loop over all nodes
	// 2) If node is not an instruction -> Resolve instructions and add to graph
	// 3) Add edge for all edges to instructions
	for (auto it = m_Graph.begin(); it != m_Graph.end(); it++) {
		switch (it->type) {
			case FUNCTION: {
				auto func = reinterpret_cast<llvm::Function *>(it->ID);
				for (auto &B : *func) {
					this->expandBasicBlock(it, &B);
				}
			}
				break;
			case BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(it->ID);
				this->expandBasicBlock(it, B);
			}
				break;
			case INSTRUCTION:
				continue;
		}
	}
}

void ConflictGraph::expandBasicBlock(stlplus::digraph<Node, Edge>::iterator it, llvm::BasicBlock *B) {
	for (auto &I : *B) {
		auto node = this->insertNode(&I, INSTRUCTION);
		auto inputs = m_Graph.inputs(it);
		auto outputs = m_Graph.outputs(it);
		for (auto arc_it = inputs.begin(); arc_it != inputs.end(); arc_it++) {
			auto value = *arc_it;
			if (value->type != PROTECTION) continue;
			auto input_node = m_Graph.arc_from(value);
			m_Graph.arc_insert(input_node, node, *value);
		}

		for (auto arc_it = outputs.begin(); arc_it != outputs.end(); arc_it++) {
			auto value = *arc_it;
			if (value->type != PROTECTION) continue;
			auto output_node = m_Graph.arc_to(value);
			m_Graph.arc_insert(node, output_node, *value);
		}
	}
}

void ConflictGraph::reduce() {
	for (auto it = m_Graph.begin(); it != m_Graph.end();) {
		switch (it->type) {
			case FUNCTION:
				it = m_Graph.erase(it);
				break;
			case BASICBLOCK:
				it = m_Graph.erase(it);
				break;
			case INSTRUCTION:
				if (m_Graph.fanin(it) == 0 && m_Graph.fanout(it) == 0) {
					it = m_Graph.erase(it);
					break;
				}
				it++;
				break;
		}
	}
}