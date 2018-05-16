#include "constraints/ConflictGraph.hpp"
#include "composition/ManifestRegistry.hpp"

using namespace llvm;
using namespace composition;

graph_t::vertex_descriptor ConflictGraph::insertNode(llvm::Value *input, NodeType type) {
	auto id = reinterpret_cast<uintptr_t>(input);

	auto result = Nodes.find(id);
	if (result != std::end(Nodes)) {
		return result->second;
	}

	auto v = boost::add_vertex(Graph);
	Graph[v] = Vertex{id, input->getName().str(), type};
	Nodes[id] = v;
	return v;
}

void ConflictGraph::removeProtection(uintptr_t protectionID) {
	Protections.erase(protectionID);

	boost::graph_traits<graph_t>::edge_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::edges(Graph);
	for (next = vi; next != vi_end; vi = next) {
		++next;
		auto v = Graph[*vi];

		if (v.type == PROTECTION) {

			if (v.protectionID == protectionID) {
				boost::remove_edge(*vi, Graph);
			}
		}
	}
}

void ConflictGraph::expandToInstructions() {
	// 1) Loop over all nodes
	// 2) If node is not an instruction -> Resolve instructions and add to graph
	// 3) Add edge for all edges to instructions
	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		Vertex v = Graph[*vi];
		switch (v.type) {
			case FUNCTION: {
				auto func = reinterpret_cast<llvm::Function *>(v.ID);
				for (auto &B : *func) {
					this->expandBasicBlockToInstructions(*vi, &B);
				}
			}
				break;
			case BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(v.ID);
				this->expandBasicBlockToInstructions(*vi, B);
			}
				break;
			case INSTRUCTION:
				continue;
		}
	}
}

void ConflictGraph::expandBasicBlockToInstructions(graph_t::vertex_descriptor it, llvm::BasicBlock *B) {
	for (auto &I : *B) {
		auto node = this->insertNode(&I, INSTRUCTION);
		replaceTarget(it, node);
	}
}

void ConflictGraph::reduceToInstructions() {
	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;
		auto it = Graph[*vi];
		switch (it.type) {
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
		}
	}
}

const graph_t &ConflictGraph::getGraph() const {
	return Graph;
}

graph_t::vertex_descriptor find_first_vertex_with_index(const int &idx, const graph_t &g)  {
	using vd = typename graph_t::vertex_descriptor;
	const auto vip = boost::vertices(g);
	const auto i = std::find_if(vip.first, vip.second, [g, idx](const vd d) {
		return boost::get(boost::get(boost::vertex_index, g), d) == idx;
	});
	if (i == vip.second) {
		std::stringstream msg;
		msg << __func__ << " : could not find vertex with index '" << idx << "'";
		throw std::invalid_argument(msg.str());
	}
	return *i;
}

void ConflictGraph::SCC() {
	unsigned long num = 0;
	unsigned long vertices = boost::num_vertices(Graph);
	while (num != vertices) {
		int idx = 0;
		boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
		for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
			boost::put(boost::vertex_index, Graph, *vi, idx++);
		}


		std::vector<int> component(boost::num_vertices(Graph)), discover_time(boost::num_vertices(Graph));
		std::vector<boost::default_color_type> color(boost::num_vertices(Graph));
		std::vector<graph_t::vertex_descriptor> root(boost::num_vertices(Graph));
		num = static_cast<unsigned long>(boost::strong_components(Graph, boost::make_iterator_property_map(component.begin(), get(boost::vertex_index, Graph)),
		                                                          root_map(boost::make_iterator_property_map(root.begin(), get(boost::vertex_index, Graph))).
						                                   discover_time_map(boost::make_iterator_property_map(discover_time.begin(), get(boost::vertex_index, Graph))).
						                                   color_map(make_iterator_property_map(color.begin(), get(boost::vertex_index, Graph)))
				));

		dbgs() << "Total number of components: " << std::to_string(num) << "\n";
		for (auto i = 0; i != num; i++) {
			std::vector<int> matches;
			for (auto j = 0; j != component.size(); j++) {
				if (component[j] == i) {
					matches.push_back(j);
				}
			}

			if (matches.size() != 1) {
				dbgs() << "Component " << std::to_string(i) << " contains cycle with " << std::to_string(matches.size()) << " elements.\n";
				handleCycle(matches);
			}
		}

		for (auto i = 0; i != component.size(); ++i) {
			dbgs() << "Vertex " << std::to_string(i) << " " << Graph[find_first_vertex_with_index(i, Graph)].name << " is in component "
			       << std::to_string(component[i]) << "\n";
		}
	}

	std::vector<uintptr_t> leftProtections;
	graph_t::edge_iterator it, it_end;
	for(std::tie(it, it_end) = boost::edges(Graph); it != it_end; it++) {
		leftProtections.push_back(Graph[*it].protectionID);
	}

	for(uintptr_t i = 0; i < ProtectionIdx; i++) {
		if(std::find(leftProtections.begin(), leftProtections.end(), i) == leftProtections.end()) {
			removeProtection(i);
			composition::ManifestRegistry::Remove(i);
		}
	}
}

void ConflictGraph::expandToFunctions() {
	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::vertices(Graph); vi != vi_end; vi++) {
		Vertex v = Graph[*vi];
		switch (v.type) {
			case FUNCTION:
				break;
			case BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(v.ID);
				this->expandBasicBlockToFunction(*vi, B);
			}
				break;
			case INSTRUCTION:
				auto I = reinterpret_cast<llvm::Instruction *>(v.ID);
				this->expandInstructionToFunction(*vi, I);
		}
	}
}

void ConflictGraph::expandBasicBlockToFunction(graph_t::vertex_descriptor it, llvm::BasicBlock *B) {
	if (B->getParent() == nullptr) {
		return;
	}
	auto func = B->getParent();
	auto funcNode = this->insertNode(func, FUNCTION);

	replaceTarget(it, funcNode);
}

void ConflictGraph::expandInstructionToFunction(graph_t::vertex_descriptor it, llvm::Instruction *I) {
	if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
		return;
	}
	auto func = I->getParent()->getParent();
	auto funcNode = this->insertNode(func, FUNCTION);

	replaceTarget(it, funcNode);
}

void ConflictGraph::replaceTarget(graph_t::vertex_descriptor src, graph_t::vertex_descriptor dst) {
	replaceTargetIncomingEdges(src, dst);
	replaceTargetOutgoingEdges(src, dst);
}

void ConflictGraph::replaceTargetIncomingEdges(graph_t::vertex_descriptor src, graph_t::vertex_descriptor dst) {
	boost::graph_traits<graph_t>::in_edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::in_edges(src, Graph); vi != vi_end; vi++) {
		auto v = Graph[*vi];
		if (v.type != PROTECTION) continue;

		auto from = boost::source(*vi, Graph);
		boost::add_edge(from, dst, v, Graph);
	}
}

void ConflictGraph::replaceTargetOutgoingEdges(graph_t::vertex_descriptor src, graph_t::vertex_descriptor dst) {
	boost::graph_traits<graph_t>::out_edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::out_edges(src, Graph); vi != vi_end; vi++) {
		auto v = Graph[*vi];
		if (v.type != PROTECTION) continue;

		auto to = boost::target(*vi, Graph);
		boost::add_edge(dst, to, v, Graph);
	}
}


void ConflictGraph::reduceToFunctions() {
	boost::graph_traits<graph_t>::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;
		auto it = Graph[*vi];
		switch (it.type) {
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
				break;
			}
		}
	}
}

void ConflictGraph::handleCycle(std::vector<int> matches) {
	dbgs() << "Handling cycle in component\n";

	graph_t::vertex_descriptor prev = nullptr;
	for (auto it = matches.begin(), it_end = matches.end(); it != it_end; it++) {
		auto vertex = find_first_vertex_with_index(*it, Graph);
		dbgs() << Graph[vertex].name << "\n";
		dbgs() << std::to_string(Graph[vertex].ID) << "\n";

		if(it == matches.begin()) {
			prev = vertex;
			continue;
		}

		boost::remove_edge(vertex, prev, Graph);
		boost::remove_edge(prev, vertex, Graph);

		prev = vertex;
	}
}
