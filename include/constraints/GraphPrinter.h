#ifndef PROJECT_GRAPH_PRINTER_H
#define PROJECT_GRAPH_PRINTER_H

#include "constraints/ConflictGraph.h"
#include "llvm/Support/Debug.h"

class GraphPrinter {
private:
	stlplus::digraph<Node, Edge> m_Graph;
private:
	void printEdges(std::unordered_map<Node, uint> map);

	std::unordered_map<Node, uint> printNodes();

public:
	explicit GraphPrinter(stlplus::digraph<Node, Edge> m_Graph) {
		this->m_Graph = m_Graph;
	}

	void dump_dot();
};

#endif //PROJECT_GRAPH_PRINTER_H
