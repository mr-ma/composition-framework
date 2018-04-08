#ifndef PROJECT_GRAPH_PRINTER_H
#define PROJECT_GRAPH_PRINTER_H

#include "constraints/ConflictGraph.hpp"
#include "llvm/Support/Debug.h"

class GraphPrinter {
private:
	graph_t Graph;
private:
	void printEdges(std::unordered_map<Vertex, uint> map);

	std::unordered_map<Vertex, uint> printNodes();

public:
	explicit GraphPrinter (graph_t graph) {
		this->Graph = graph;
	}

	void dump_dot();
};

#endif //PROJECT_GRAPH_PRINTER_H
