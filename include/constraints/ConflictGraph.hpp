#ifndef PROJECT_GRAPH_H
#define PROJECT_GRAPH_H

#include <string>
#include <unordered_map>
#include <stack>
#include <llvm/IR/Function.h>
#include "llvm/ADT/Twine.h"
#include "llvm/Support/Debug.h"
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/strong_components.hpp>
#include "composition/Manifest.hpp"

struct Vertex {
	uintptr_t ID;
	std::string name;
	NodeType type;

	bool operator==(const Vertex &other) const {
		return (ID == other.ID);
	}

	bool operator<(const Vertex &other) const {
		return (ID < other.ID);
	}
};

enum EdgeType {
	HIERARCHY,
	PROTECTION
};

struct Edge {
	uintptr_t protectionID;
	std::string name;
	EdgeType type;
};

struct Protection {
	llvm::Value *from;
	NodeType fromType;
	llvm::Value *to;
	NodeType toType;
};

namespace std {
	template<>
	struct hash<Vertex> {
		size_t operator()(const Vertex &pt) const {
			return pt.ID;
		}
	};
}

typedef boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS, boost::property<boost::vertex_index_t, int, Vertex>, Edge> graph_t;
using vertex_t = boost::graph_traits<graph_t>::vertex_descriptor;
using edge_t   = boost::graph_traits<graph_t>::edge_descriptor;

class ConflictGraph {
private:
	std::unordered_map<uintptr_t, graph_t::vertex_descriptor> Nodes;
	graph_t Graph;
	uintptr_t ProtectionIdx;
	std::unordered_map<uintptr_t, Protection> Protections;

private:
	graph_t::vertex_descriptor insertNode(llvm::Value *node, NodeType type);

	void expandBasicBlockToInstructions(graph_t::vertex_descriptor B, llvm::BasicBlock *pBlock);

	void expandInstructionToFunction(graph_t::vertex_descriptor it, llvm::Instruction *I);

	void expandBasicBlockToFunction(graph_t::vertex_descriptor it, llvm::BasicBlock *B);

	void replaceTarget(graph_t::vertex_descriptor src, graph_t::vertex_descriptor dst);

	void replaceTargetIncomingEdges(graph_t::vertex_descriptor src, graph_t::vertex_descriptor dst);

	void replaceTargetOutgoingEdges(graph_t::vertex_descriptor src, graph_t::vertex_descriptor dst);

public:
	ConflictGraph() = default;

	ConflictGraph(const ConflictGraph &that) = delete;

	const graph_t &getGraph() const;

	template<typename T, typename S>
	uintptr_t addProtection(std::string name, T protector, S protectee) {
		// Two functions, a protector and a protectee form an edge
		// The direction is from the protectee to the protector to indicate the information flow.
		auto dstNode = this->insertNode(protector, TypeToNodeType<T>().value);
		auto srcNode = this->insertNode(protectee, TypeToNodeType<S>().value);

		boost::add_edge(srcNode, dstNode, Edge{ProtectionIdx, name, PROTECTION}, Graph);
		Protections[ProtectionIdx] = Protection{protector, TypeToNodeType<T>().value, protectee, TypeToNodeType<S>().value};
		return ProtectionIdx++;
	}

	template<typename T, typename S>
	void addHierarchy(T parent, S child) {
		auto srcNode = this->insertNode(parent, TypeToNodeType<T>().value);
		auto dstNode = this->insertNode(child, TypeToNodeType<S>().value);
		boost::add_edge(srcNode, dstNode, Edge{UINTPTR_MAX, "", HIERARCHY}, Graph);
	}

	void removeProtection(uintptr_t protectionID);

	void expandToInstructions();

	void reduceToInstructions();

	void expandToFunctions();

	void reduceToFunctions();

	void SCC();
};


#endif //PROJECT_GRAPH_H
