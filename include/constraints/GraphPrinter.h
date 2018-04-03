#ifndef PROJECT_GRAPH_PRINTER_H
#define PROJECT_GRAPH_PRINTER_H

#include "constraints/ConflictGraph.h"
#include "llvm/Support/Debug.h"

class GraphPrinter {
private:
	Graph m_Graph;
private:
	void printEdges(std::unordered_map<Vertex, uint> map);

	std::unordered_map<Vertex, uint> printNodes();

public:
	explicit GraphPrinter(Graph m_Graph) {
		this->m_Graph = m_Graph;
	}

	void dump_dot();
};

#endif //PROJECT_GRAPH_PRINTER_H
