#ifndef COMPOSITION_FRAMEWORK_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_PROTECTIONGRAPH_HPP

#include <llvm/IR/Value.h>
#include <composition/graph/graph.hpp>
#include <composition/graph/protection.hpp>

namespace composition {
	class ProtectionGraph {
	private:
		graph_t Graph{};
		edge_idx_t ProtectionIdx{};
		std::unordered_map<edge_idx_t, Protection> Protections{};

	private:
		vd insertNode(llvm::Value *node, vertex_type type);

		void expandBasicBlockToInstructions(vd B, llvm::BasicBlock *pBlock);

		void expandInstructionToFunction(vd it, llvm::Instruction *I);

		void expandBasicBlockToFunction(vd it, llvm::BasicBlock *B);

		void replaceTarget(vd src, vd dst);

		void replaceTargetIncomingEdges(vd src, vd dst);

		void replaceTargetOutgoingEdges(vd src, vd dst);

	public:
		ProtectionGraph() = default;

		ProtectionGraph(const ProtectionGraph &that) = delete;

		ProtectionGraph(const ProtectionGraph &&that) noexcept : ProtectionIdx(that.ProtectionIdx),
		                                                         Protections(that.Protections),
		                                                         Graph(that.Graph) {
		};

		ProtectionGraph &operator=(ProtectionGraph &&) = default;

		graph_t &getGraph();

		template<typename T, typename S>
		uintptr_t addProtection(const std::string &name, T protector, S protectee) {
			// Two functions, a protector and a protectee form an edge
			// The direction is from the protectee to the protector to indicate the information flow.
			return addProtection(name, protector, TypeToNodeType<T>().value, protectee, protectee, TypeToNodeType<S>().value);
		}

		uintptr_t
		addProtection(const std::string &name, llvm::Value *protector, vertex_type protectorType, llvm::Value *protectee, vertex_type protecteeType) {
			// Two functions, a protector and a protectee form an edge
			// The direction is from the protectee to the protector to indicate the information flow.
			auto dstNode = this->insertNode(protector, protectorType);
			auto srcNode = this->insertNode(protectee, protecteeType);

			auto edge = boost::add_edge(srcNode, dstNode, Graph);
			assert(edge.second);
			set_edge_property(boost::edge_index, edge.first, ProtectionIdx, Graph);
			set_edge_property(boost::edge_name, edge.first, name, Graph);
			set_edge_property(boost::edge_type, edge.first, DEPENDENCY, Graph);
			Protections[ProtectionIdx] = Protection(protector, protectee);
			return ProtectionIdx++;
		}

		template<typename T, typename S>
		void addHierarchy(T parent, S child) {
			auto srcNode = this->insertNode(parent, TypeToNodeType<T>().value);
			auto dstNode = this->insertNode(child, TypeToNodeType<S>().value);

			auto edge = boost::add_edge(srcNode, dstNode, Graph);
			assert(edge.second);
			set_edge_property(boost::edge_type, edge.first, CFG, Graph);
		}

		void removeProtection(uintptr_t protectionID);

		void expandToInstructions();

		void reduceToInstructions();

		void expandToFunctions();

		void reduceToFunctions();

		void SCC();

		void handleCycle(std::vector<int> matches);
	};
}
#endif //COMPOSITION_FRAMEWORK_PROTECTIONGRAPH_HPP