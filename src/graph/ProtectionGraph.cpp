#include <cstdint>
#include <unordered_set>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/graph/strong_components.hpp>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/util/functions.hpp>

using namespace llvm;
using namespace composition;

vd_t ProtectionGraph::insertNode(llvm::Value *input, vertex_type type) {
	assertType(input, type);

	auto idx = reinterpret_cast<uintptr_t>(input);

	if (has_vertex_with_property(&vertex_t::index, idx, Graph)) {
		return find_first_vertex_with_property(&vertex_t::index, idx, Graph);
	}

	std::string name;
	if (!input->hasName()) {
		if (isa<Instruction>(input)) {
			name = "I";
		} else if (isa<BasicBlock>(input)) {
			name = "BB";
		}
	} else {
		name = input->getName();
	}

	return boost::add_vertex(vertex_t(idx, name, type, {}), Graph);
}

void ProtectionGraph::removeProtection(uintptr_t protectionID) {
	Protections.erase(protectionID);

	graph_t::edge_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::edges(Graph);
	for (next = vi; next != vi_end; vi = next) {
		++next;

		auto e = Graph[*vi];
		if (e.type == edge_type::DEPENDENCY) {
			if (e.index == protectionID) {
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
		auto v = Graph[*vi];

		switch (v.type) {
			case vertex_type::FUNCTION: {
				auto func = reinterpret_cast<llvm::Function *>(v.index);
				for (auto &B : *func) {
					this->expandBasicBlockToInstructions(*vi, &B);
				}
			}
				break;
			case vertex_type::BASICBLOCK: {
				auto B = reinterpret_cast<llvm::BasicBlock *>(v.index);
				this->expandBasicBlockToInstructions(*vi, B);
			}
				break;
			case vertex_type::INSTRUCTION:
				break;
			case vertex_type::VALUE:
				break;
			case vertex_type::UNKNOWN:
				break;
		}
	}
}

void ProtectionGraph::expandBasicBlockToInstructions(vd_t it, llvm::BasicBlock *B) {
	for (auto &I : *B) {
		auto node = this->insertNode(&I, vertex_type::INSTRUCTION);
		replaceTarget(it, node);
	}
}

void ProtectionGraph::reduceToInstructions() {
	graph_t::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;
		auto v = Graph[*vi];
		switch (v.type) {
			case vertex_type::FUNCTION:
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
				break;
			case vertex_type::BASICBLOCK:
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
				break;
			case vertex_type::INSTRUCTION: {
				if (boost::out_degree(*vi, Graph) == 0 && boost::in_degree(*vi, Graph) == 0) {
					boost::remove_vertex(*vi, Graph);
				}
				break;
			}
			case vertex_type::VALUE:
				break;
			case vertex_type::UNKNOWN:
				break;
		}
	}
}

graph_t &ProtectionGraph::getGraph() {
	return Graph;
}

void ProtectionGraph::SCC() {
	unsigned long num = 0;
	unsigned long vertices = boost::num_vertices(Graph);
	while (num != vertices) {

		std::map<vd_t, size_t> index;
		auto pmap = boost::make_assoc_property_map(index);

		int i = 0;
		for (auto vd : boost::make_iterator_range(boost::vertices(Graph))) {
			put(pmap, vd, i++);
		}


		std::vector<int> component(vertices), discover_time(vertices);
		std::vector<boost::default_color_type> color(vertices);
		std::vector<graph_t::vertex_descriptor> root(vertices);
		num = static_cast<unsigned long>(boost::strong_components(Graph, boost::iterator_property_map(component.begin(), pmap),
		                                                          root_map(boost::make_iterator_property_map(root.begin(), pmap)).
				                                                          discover_time_map(boost::make_iterator_property_map(discover_time.begin(),
				                                                                                                              pmap)).
				                                                          color_map(make_iterator_property_map(color.begin(), pmap))
		));

		//TODO the generated resulting graph file looks wrong
		//it is possible that SCC produces an incorrect result and it needs to be checked!
		dbgs() << "Total number of components: " << std::to_string(num) << "\n";
		for (auto i = 0; i != num; i++) {
			std::vector<vd_t> matches;
			for (auto &el : index) {
				if (component[el.second] == i) {
					matches.push_back(el.first);
				}
			}

			if (matches.size() != 1) {
				dbgs() << "Component " << std::to_string(i) << " contains cycle with " << std::to_string(matches.size() + 1) << " elements.\n";
				handleCycle(matches);
			}
		}

		//TODO If we want to access the vertices sorted by there aritificial index from above
		//Then we need to sort it according to the next 2 blocks. Otherwise, we could use index map directly.
		std::vector<std::pair<vd_t, size_t>> pairs;
		for (auto it = index.begin(); it != index.end(); it++) {
			pairs.push_back(*it);
		}

		sort(pairs.begin(), pairs.end(), [=](std::pair<vd_t, size_t> &a, std::pair<vd_t, size_t> &b) {
			return a.second < b.second;
		});


		for (auto &el : pairs) {
			auto v = Graph[el.first];
			dbgs() << "Vertex " << std::to_string(el.second) << " " << v.name << " is in component " << std::to_string(component[el.second]) << "\n";
		}
	}

	std::vector<uintptr_t> leftProtections;
	graph_t::edge_iterator it, it_end;
	for (std::tie(it, it_end) = boost::edges(Graph); it != it_end; it++) {
		leftProtections.push_back(get_edge_property(&edge_t::index, *it, Graph));
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
		auto v = Graph[*vi];
		switch (v.type) {
			case vertex_type::FUNCTION:
				break;
			case vertex_type::BASICBLOCK: {
				const auto &B = reinterpret_cast<llvm::BasicBlock *>(v.index);
				this->expandBasicBlockToFunction(*vi, B);
			}
				break;
			case vertex_type::INSTRUCTION: {
				const auto &I = reinterpret_cast<llvm::Instruction *>(v.index);
				this->expandInstructionToFunction(*vi, I);
			}
				break;
			case vertex_type::VALUE:
				break;
			case vertex_type::UNKNOWN:
				break;
		}
	}
}

void ProtectionGraph::expandBasicBlockToFunction(vd_t it, llvm::BasicBlock *B) {
	if (B->getParent() == nullptr) {
		return;
	}
	auto func = B->getParent();
	auto funcNode = this->insertNode(func, vertex_type::FUNCTION);

	replaceTarget(it, funcNode);
}

void ProtectionGraph::expandInstructionToFunction(vd_t it, llvm::Instruction *I) {
	if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
		return;
	}
	auto func = I->getParent()->getParent();
	auto funcNode = this->insertNode(func, vertex_type::FUNCTION);

	replaceTarget(it, funcNode);
}

void ProtectionGraph::replaceTarget(vd_t src, vd_t dst) {
	replaceTargetIncomingEdges(src, dst);
	replaceTargetOutgoingEdges(src, dst);
}

void ProtectionGraph::replaceTargetIncomingEdges(vd_t src, vd_t dst) {
	graph_t::in_edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::in_edges(src, Graph); vi != vi_end; vi++) {
		auto e = Graph[*vi];
		if (e.type != edge_type::DEPENDENCY) continue;

		auto from = boost::source(*vi, Graph);
		auto edge = boost::add_edge(from, dst, Graph);
		assert(edge.second);

		edge_t eNew(e.index, e.name, e.type);
		Graph[edge.first] = eNew;
	}
}

void ProtectionGraph::replaceTargetOutgoingEdges(vd_t src, vd_t dst) {
	graph_t::out_edge_iterator vi, vi_end;
	for (std::tie(vi, vi_end) = boost::out_edges(src, Graph); vi != vi_end; vi++) {
		auto e = Graph[*vi];
		if (e.type != edge_type::DEPENDENCY) continue;

		auto to = boost::target(*vi, Graph);
		auto edge = boost::add_edge(dst, to, Graph);
		assert(edge.second);

		edge_t eNew(e.index, e.name, e.type);
		Graph[edge.first] = eNew;
	}
}


void ProtectionGraph::reduceToFunctions() {
	graph_t::vertex_iterator vi, vi_end, next;
	std::tie(vi, vi_end) = boost::vertices(Graph);
	for (next = vi; vi != vi_end; vi = next) {
		++next;

		auto v = Graph[*vi];

		switch (v.type) {
			case vertex_type::FUNCTION:
				if (boost::out_degree(*vi, Graph) == 0 && boost::in_degree(*vi, Graph) == 0) {
					boost::remove_vertex(*vi, Graph);
				}
				break;
			case vertex_type::BASICBLOCK:
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
				break;
			case vertex_type::INSTRUCTION: {
				boost::clear_vertex(*vi, Graph);
				boost::remove_vertex(*vi, Graph);
			}
				break;
			case vertex_type::VALUE:
				break;
			case vertex_type::UNKNOWN:
				break;

		}
	}
}

void ProtectionGraph::handleCycle(std::vector<vd_t> matches) {
	dbgs() << "Handling cycle in component\n";

	vd_t prev = nullptr;
	for (auto it = matches.begin(), it_end = matches.end(); it != it_end; it++) {
		vd_t vd = *it;
		auto v = Graph[vd];

		dbgs() << v.name << "\n";
		dbgs() << std::to_string(v.index) << "\n";

		if (it == matches.begin()) {
			prev = vd;
			continue;
		}

		boost::remove_edge(vd, prev, Graph);
		boost::remove_edge(prev, vd, Graph);

		prev = vd;
	}
}

uintptr_t
ProtectionGraph::addProtection(const std::string &name, llvm::Value *protector, vertex_type protectorType, llvm::Value *protectee, vertex_type protecteeType) {
	// Two functions, a protector and a protectee form an edge
	// The direction is from the protectee to the protector to indicate the information flow.
	auto dstNode = this->insertNode(protector, protectorType);
	auto srcNode = this->insertNode(protectee, protecteeType);

	auto edge = boost::add_edge(srcNode, dstNode, Graph);
	assert(edge.second);
	Graph[edge.first] = edge_t{ProtectionIdx, name, edge_type::DEPENDENCY};
	Protections[ProtectionIdx] = Protection(protector, protectee);
	return ProtectionIdx++;
}
