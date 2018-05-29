#include <llvm/Support/Debug.h>
#include <boost/graph/strong_components.hpp>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/ManifestRegistry.hpp>

using namespace llvm;
using namespace composition;

vd ProtectionGraph::insertNode(llvm::Value *input, vertex_type type) {
	auto idx = reinterpret_cast<uintptr_t>(input);

	if (has_vertex_with_property(boost::vertex_index, idx, Graph)) {
		return find_first_vertex_with_property(boost::vertex_index, idx, Graph);
	}

	auto v = boost::add_vertex(Graph);
	set_vertex_property(boost::vertex_index, v, idx, Graph);
	set_vertex_property(boost::vertex_name, v, input->getName().str(), Graph);
	set_vertex_property(boost::vertex_type, v, type, Graph);
	return v;
}

void ProtectionGraph::removeProtection(uintptr_t protectionID) {
	Protections.erase(protectionID);

	graph_t::edge_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::edges(Graph);
	for (next = vi; next != vi_end; vi = next) {
		++next;

		auto type = get_edge_property(boost::edge_type, *vi, Graph);
		if (type == PROTECTION) {
			auto idx = get_edge_property(boost::edge_index, *vi, Graph);
			if (idx == protectionID) {
				boost::remove_edge(*vi, Graph);
			}
		}
	}
}

void ProtectionGraph::expandToInstructions() {
	// 1) Loop over all nodes
	// 2) If node is not an instruction -> Resolve instructions and add to graph
	// 3) Add edge for all edges to instructions
	graph_t::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		auto type = get_vertex_property(boost::vertex_type, *vi, Graph);
		auto idx = get_vertex_property(boost::vertex_index, *vi, Graph);

		switch (type) {
			case FUNCTION: {
				auto func = reinterpret_cast<llvm::Function *>(idx);
				for (auto &B : *func) {
					this->expandBasicBlockToInstructions(*vi, &B);
				}
			}
				break;
			case BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(idx);
				this->expandBasicBlockToInstructions(*vi, B);
			}
				break;
			case INSTRUCTION:
				break;
			case VALUE:
				break;
		}
	}
}

void ProtectionGraph::expandBasicBlockToInstructions(vd it, llvm::BasicBlock *B) {
	for (auto &I : *B) {
		auto node = this->insertNode(&I, INSTRUCTION);
		replaceTarget(it, node);
	}
}

void ProtectionGraph::reduceToInstructions() {
	graph_t::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;
		auto type = get_vertex_property(boost::vertex_type, *vi, Graph);
		switch (type) {
			case FUNCTION:
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
				break;
			case BASICBLOCK:
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
				break;
			case INSTRUCTION: {
				if (boost::out_degree(*vi, Graph) == 0 && boost::in_degree(*vi, Graph) == 0) {
					boost::remove_vertex(*vi, Graph);
				}
				break;
			}
			case VALUE:
				break;
		}
	}
}

const graph_t &ProtectionGraph::getGraph() const {
	return Graph;
}

void ProtectionGraph::SCC() {
	unsigned long num = 0;
	unsigned long vertices = boost::num_vertices(Graph);
	while (num != vertices) {
		int idx = 0;
		boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
		for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
			set_vertex_property(boost::vertex_index, *vi, idx++, Graph);
		}


		std::vector<int> component(vertices), discover_time(vertices);
		std::vector<boost::default_color_type> color(vertices);
		std::vector<graph_t::vertex_descriptor> root(vertices);
		num = static_cast<unsigned long>(boost::strong_components(Graph, boost::make_iterator_property_map(component.begin(), get(boost::vertex_index, Graph)),
		                                                          root_map(boost::make_iterator_property_map(root.begin(), get(boost::vertex_index, Graph))).
				                                                          discover_time_map(boost::make_iterator_property_map(discover_time.begin(),
				                                                                                                              get(boost::vertex_index, Graph))).
				                                                          color_map(make_iterator_property_map(color.begin(), get(boost::vertex_index, Graph)))
		));

		dbgs() << "Total number of components: " << std::to_string(num) << "\n";
		for (auto i = 0; i != num; i++) {
			std::vector<int> matches;
			for (auto j = 0; j != vertices; j++) {
				if (component[j] == i) {
					matches.push_back(j);
				}
			}

			if (matches.size() != 1) {
				dbgs() << "Component " << std::to_string(i) << " contains cycle with " << std::to_string(matches.size()) << " elements.\n";
				handleCycle(matches);
			}
		}

		for (auto i = 0; i != vertices; ++i) {
			auto str = get_vertex_property(boost::vertex_name, find_first_vertex_with_property(boost::vertex_index, i, Graph), Graph);
			dbgs() << "Vertex " << std::to_string(i) << " " << str << " is in component " << std::to_string(component[i]) << "\n";
		}
	}

	std::vector<uintptr_t> leftProtections;
	graph_t::edge_iterator it, it_end;
	for (std::tie(it, it_end) = boost::edges(Graph); it != it_end; it++) {
		leftProtections.push_back(get_edge_property(boost::edge_index, *it, Graph));
	}

	for (uintptr_t i = 0; i < ProtectionIdx; i++) {
		if (std::find(leftProtections.begin(), leftProtections.end(), i) == leftProtections.end()) {
			removeProtection(i);
			composition::ManifestRegistry::Remove(i);
		}
	}
}

void ProtectionGraph::expandToFunctions() {
	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		auto type = get_vertex_property(boost::vertex_type, *vi, Graph);
		auto idx = get_vertex_property(boost::vertex_index, *vi, Graph);
		switch (type) {
			case FUNCTION:
				break;
			case BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(idx);
				this->expandBasicBlockToFunction(*vi, B);
			}
				break;
			case INSTRUCTION: {
				auto I = reinterpret_cast<llvm::Instruction *>(idx);
				this->expandInstructionToFunction(*vi, I);
			}
				break;
			case VALUE:
				break;
		}
	}
}

void ProtectionGraph::expandBasicBlockToFunction(vd it, llvm::BasicBlock *B) {
	if (B->getParent() == nullptr) {
		return;
	}
	auto func = B->getParent();
	auto funcNode = this->insertNode(func, FUNCTION);

	replaceTarget(it, funcNode);
}

void ProtectionGraph::expandInstructionToFunction(vd it, llvm::Instruction *I) {
	if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
		return;
	}
	auto func = I->getParent()->getParent();
	auto funcNode = this->insertNode(func, FUNCTION);

	replaceTarget(it, funcNode);
}

void ProtectionGraph::replaceTarget(vd src, vd dst) {
	replaceTargetIncomingEdges(src, dst);
	replaceTargetOutgoingEdges(src, dst);
}

void ProtectionGraph::replaceTargetIncomingEdges(vd src, vd dst) {
	graph_t::in_edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::in_edges(src, Graph); vi != vi_end; vi++) {
		auto type = get_edge_property(boost::edge_type, *vi, Graph);
		if (type != PROTECTION) continue;

		auto idx = get_edge_property(boost::edge_index, *vi, Graph);
		auto name = get_edge_property(boost::edge_name, *vi, Graph);

		auto from = boost::source(*vi, Graph);
		auto edge = boost::add_edge(from, dst, Graph);
		assert(edge.second);

		set_edge_property(boost::edge_index, edge.first, idx, Graph);
		set_edge_property(boost::edge_name, edge.first, name, Graph);
		set_edge_property(boost::edge_type, edge.first, type, Graph);
	}
}

void ProtectionGraph::replaceTargetOutgoingEdges(vd src, vd dst) {
	graph_t::out_edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::out_edges(src, Graph); vi != vi_end; vi++) {
		auto type = get_edge_property(boost::edge_type, *vi, Graph);
		if (type != PROTECTION) continue;

		auto idx = get_edge_property(boost::edge_index, *vi, Graph);
		auto name = get_edge_property(boost::edge_name, *vi, Graph);

		auto to = boost::target(*vi, Graph);
		auto edge = boost::add_edge(dst, to, Graph);
		assert(edge.second);

		set_edge_property(boost::edge_index, edge.first, idx, Graph);
		set_edge_property(boost::edge_name, edge.first, name, Graph);
		set_edge_property(boost::edge_type, edge.first, type, Graph);
	}
}


void ProtectionGraph::reduceToFunctions() {
	graph_t::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;

		auto type = get_vertex_property(boost::vertex_type, *vi, Graph);

		switch (type) {
			case FUNCTION:
				if (boost::out_degree(*vi, Graph) == 0 && boost::in_degree(*vi, Graph) == 0) {
					boost::remove_vertex(*vi, Graph);
				}
				break;
			case BASICBLOCK:
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
				break;
			case INSTRUCTION: {
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
			}
				break;
			case VALUE:
				break;

		}
	}
}

void ProtectionGraph::handleCycle(std::vector<int> matches) {
	dbgs() << "Handling cycle in component\n";

	vd prev = nullptr;
	for (auto it = matches.begin(), it_end = matches.end(); it != it_end; it++) {
		auto vertex = find_first_vertex_with_property(boost::vertex_index, *it, Graph);
		auto name = get_vertex_property(boost::vertex_name, vertex, Graph);
		auto idx = get_vertex_property(boost::vertex_index, vertex, Graph);

		dbgs() << name << "\n";
		dbgs() << std::to_string(idx) << "\n";

		if (it == matches.begin()) {
			prev = vertex;
			continue;
		}

		boost::remove_edge(vertex, prev, Graph);
		boost::remove_edge(prev, vertex, Graph);

		prev = vertex;
	}
}
