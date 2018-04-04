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
enum NodeType {
	FUNCTION,
	BASICBLOCK,
	INSTRUCTION
};

struct Vertex {
	uintptr_t ID;
	std::string name;
	NodeType type;

	bool operator==(const Vertex &other) const {
		return (ID == other.ID);
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

template<typename T>
struct TypeToNodeType;
template<>
struct TypeToNodeType<llvm::Function *> { static constexpr NodeType value = FUNCTION; };
template<>
struct TypeToNodeType<llvm::BasicBlock *> { static constexpr NodeType value = BASICBLOCK; };
template<>
struct TypeToNodeType<llvm::Instruction *> { static constexpr NodeType value = INSTRUCTION; };

typedef boost::adjacency_list<boost::listS , boost::listS, boost::bidirectionalS, boost::property<boost::vertex_index_t, int, Vertex>, Edge> Graph;
using vertex_t = boost::graph_traits<Graph>::vertex_descriptor;
using edge_t   = boost::graph_traits<Graph>::edge_descriptor;

class ConflictGraph {
private:
	std::unordered_map<uintptr_t, Graph::vertex_descriptor> m_Nodes;
	Graph m_Graph;
	uintptr_t m_ProtectionID;
	std::unordered_map<uintptr_t, Protection> m_Protections;

private:
	Graph::vertex_descriptor insertNode(llvm::Value *node, NodeType type);

	void expandBasicBlock(Graph::vertex_descriptor B, llvm::BasicBlock *pBlock);

public:
	const Graph &getGraph() const;

	template<typename T, typename S>
	uintptr_t addProtection(std::string name, T protector, S protectee) {
		// Two functions, a protector and a protectee form an edge
		// The direction is from the protectee to the protector to indicate the information flow.
		auto dstNode = this->insertNode(protector, TypeToNodeType<T>().value);
		auto srcNode = this->insertNode(protectee, TypeToNodeType<S>().value);

		boost::add_edge(srcNode, dstNode, Edge{m_ProtectionID, name, PROTECTION}, m_Graph);
		m_Protections[m_ProtectionID] = Protection{protector, TypeToNodeType<T>().value, protectee, TypeToNodeType<S>().value};
		return m_ProtectionID++;
	}

	template<typename T, typename S>
	void addHierarchy(T parent, S child) {
		auto srcNode = this->insertNode(parent, TypeToNodeType<T>().value);
		auto dstNode = this->insertNode(child, TypeToNodeType<S>().value);
		boost::add_edge(srcNode, dstNode, Edge{UINTPTR_MAX, "", HIERARCHY}, m_Graph);
	}

	void removeProtection(uintptr_t protectionID);

	void expand();

	void reduce();

	void SCC();
};


#endif //PROJECT_GRAPH_H
