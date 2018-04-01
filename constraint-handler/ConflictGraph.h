#ifndef PROJECT_GRAPH_H
#define PROJECT_GRAPH_H

#include <string>
#include <unordered_map>
#include <llvm/IR/Function.h>
#include "llvm/ADT/Twine.h"
#include "llvm/Support/Debug.h"
#include "stlplus/digraph.hpp"

enum NodeType {
	FUNCTION,
	BASICBLOCK,
	INSTRUCTION
};

struct Node {
	uintptr_t ID;
	std::string name;
	NodeType type;

	bool operator==(const Node &other) const {
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
	struct hash<Node> {
		size_t operator()(const Node &pt) const {
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


class ConflictGraph {
private:
	std::unordered_map<uintptr_t, stlplus::digraph<Node, Edge>::iterator> m_Nodes;
	stlplus::digraph<Node, Edge> m_Graph;
	uintptr_t m_ProtectionID;
	std::unordered_map<uintptr_t, Protection> m_Protections;

	std::unordered_map<Node, uint> printNodes();

	void printEdges(std::unordered_map<Node, uint> map);

	stlplus::digraph<Node, Edge>::iterator insertNode(llvm::Value *node, NodeType type);

	void expandBasicBlock(stlplus::digraph<Node, Edge>::iterator B, llvm::BasicBlock *pBlock);
public:

	void dump_dot();

	template<typename T, typename S>
	uintptr_t addProtection(std::string name, T protector, S protectee) {
		// Two functions, a protector and a protectee form an edge
		// The direction is from the protectee to the protector to indicate the information flow.
		auto dstNode = this->insertNode(protector, TypeToNodeType<T>().value);
		auto srcNode = this->insertNode(protectee, TypeToNodeType<S>().value);
		m_Graph.arc_insert(srcNode, dstNode, Edge{m_ProtectionID, name, PROTECTION});
		m_Protections[m_ProtectionID] = Protection{protector, TypeToNodeType<T>().value, protectee, TypeToNodeType<S>().value};
		return m_ProtectionID++;
	}

	template<typename T, typename S>
	void addHierarchy(T parent, S child) {
		auto srcNode = this->insertNode(parent, TypeToNodeType<T>().value);
		auto dstNode = this->insertNode(child, TypeToNodeType<S>().value);
		m_Graph.arc_insert(srcNode, dstNode, Edge{UINTPTR_MAX, "", HIERARCHY});
	}

	void removeProtection(uintptr_t protectionID);

	void expand();

	void reduce();
};


#endif //PROJECT_GRAPH_H
