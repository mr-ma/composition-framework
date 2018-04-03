#include "constraints/ConflictGraph.h"

using namespace llvm;

unsigned long ConflictGraph::insertNode(llvm::Value *input, NodeType type) {
	auto id = reinterpret_cast<uintptr_t>(input);

	auto result = m_Nodes.find(id);
	if (result != m_Nodes.end()) {
		return result->second;
	}

	m_Nodes[id] = boost::add_vertex(Vertex{id, input->getName().str(), type}, m_Graph);
	return m_Nodes[id];
}

void ConflictGraph::removeProtection(uintptr_t protectionID) {
	m_Protections.erase(protectionID);

	boost::graph_traits<Graph>::edge_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::edges(m_Graph);
	for (next = vi; next != vi_end; vi = next) {
		++next;
		auto v = m_Graph[*vi];

		if (v.type == PROTECTION) {

			if (v.protectionID == protectionID) {
				boost::remove_edge(*vi, m_Graph);
			}
		}
	}
}

void ConflictGraph::expand() {
	// 1) Loop over all nodes
	// 2) If node is not an instruction -> Resolve instructions and add to graph
	// 3) Add edge for all edges to instructions
	boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(m_Graph); vi != vi_end; vi++) {
		Vertex v = m_Graph[*vi];
		switch (v.type) {
			case FUNCTION: {
				auto func = reinterpret_cast<llvm::Function *>(v.ID);
				for (auto &B : *func) {
					this->expandBasicBlock(vi, &B);
				}
			}
				break;
			case BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(v.ID);
				this->expandBasicBlock(vi, B);
			}
				break;
			case INSTRUCTION:
				continue;
		}
	}
}

void ConflictGraph::expandBasicBlock(boost::range_detail::integer_iterator<unsigned long> it, llvm::BasicBlock *B) {
	for (auto &I : *B) {
		auto node = this->insertNode(&I, INSTRUCTION);

		{
			boost::graph_traits<Graph>::in_edge_iterator vi, vi_end;
			for (std::tie(vi, vi_end) = boost::in_edges(*it, m_Graph); vi != vi_end; vi++) {
				auto v = m_Graph[*vi];
				if (v.type != PROTECTION) continue;

				auto from = boost::source(*vi, m_Graph);
				boost::add_edge(from, node, v, m_Graph);
			}
		}


		{
			boost::graph_traits<Graph>::out_edge_iterator vi, vi_end;
			for (std::tie(vi, vi_end) = boost::out_edges(*it, m_Graph); vi != vi_end; vi++) {
				auto v = m_Graph[*vi];
				if (v.type != PROTECTION) continue;

				auto to = boost::target(*vi, m_Graph);
				boost::add_edge(node, to, v, m_Graph);
			}
		}
	}
}

void ConflictGraph::reduce() {
	//TODO: This algorithm is O(N!), fix that asap.
	boost::graph_traits<Graph>::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(m_Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;
		auto it = m_Graph[*vi];
		switch (it.type) {
			case FUNCTION:
				boost::clear_vertex(*vi, m_Graph);
				boost::remove_vertex(*vi, m_Graph);
				std::tie(next, vi_end) = boost::vertices(m_Graph);
				break;
			case BASICBLOCK:
				boost::clear_vertex(*vi, m_Graph);
				boost::remove_vertex(*vi, m_Graph);
				std::tie(next, vi_end) = boost::vertices(m_Graph);
				break;
			case INSTRUCTION: {
				if (boost::out_degree(*vi, m_Graph) == 0 && boost::in_degree(*vi, m_Graph) == 0) {
					boost::remove_vertex(*vi, m_Graph);
					std::tie(next, vi_end) = boost::vertices(m_Graph);
				}
				break;
			}
		}
	}
}

const Graph &ConflictGraph::getGraph() const {
	return m_Graph;
}
