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
		vd_t insertNode(llvm::Value *node, vertex_type type);

		void expandBasicBlockToInstructions(vd_t B, llvm::BasicBlock *pBlock);

		void expandInstructionToFunction(vd_t it, llvm::Instruction *I);

		void expandBasicBlockToFunction(vd_t it, llvm::BasicBlock *B);

		void replaceTarget(vd_t src, vd_t dst);

		void replaceTargetIncomingEdges(vd_t src, vd_t dst);

		void replaceTargetOutgoingEdges(vd_t src, vd_t dst);

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
			return addProtection(name, protector, LLVMToVertexType<T>().value, protectee, protectee, LLVMToVertexType<S>().value);
		}

		uintptr_t
		addProtection(const std::string &name, llvm::Value *protector, vertex_type protectorType, llvm::Value *protectee, vertex_type protecteeType);

		template<typename T, typename S>
		uintptr_t addCFG(T parent, S child) {
			auto srcNode = this->insertNode(parent, LLVMToVertexType<T>().value);
			auto dstNode = this->insertNode(child, LLVMToVertexType<S>().value);

			auto edge = boost::add_edge(srcNode, dstNode, Graph);
			assert(edge.second);
			Graph[edge.first] = edge_t{ProtectionIdx, "CFG", edge_type::CFG};
			Protections[ProtectionIdx] = Protection(parent, child);
			return ProtectionIdx++;
		}

		void removeProtection(uintptr_t protectionID);

		void expandToInstructions();

		void reduceToInstructions();

		void expandToFunctions();

		void reduceToFunctions();

		void SCC();

		void handleCycle(std::vector<vd_t> matches);
	};
}
#endif //COMPOSITION_FRAMEWORK_PROTECTIONGRAPH_HPP
