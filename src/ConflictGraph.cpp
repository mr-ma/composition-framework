#include "constraints/ConflictGraph.h"

using namespace llvm;

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

const stlplus::digraph<Node, Edge> &ConflictGraph::getGraph() const {
	return m_Graph;
}
