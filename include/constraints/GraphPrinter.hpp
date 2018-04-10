#ifndef PROJECT_GRAPH_PRINTER_H
#define PROJECT_GRAPH_PRINTER_H

#include "constraints/ConflictGraph.hpp"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FileSystem.h"

class GraphPrinter {
private:
	graph_t Graph;
private:
	void printEdges(std::unordered_map<Vertex, uint> map, llvm::raw_ostream &stream);

	std::unordered_map<Vertex, uint> printNodes(llvm::raw_ostream &stream);

public:
	explicit GraphPrinter(graph_t graph) {
		this->Graph = graph;
	}

	void dump_dot(llvm::raw_ostream &stream);

	void dump_dot(std::string fileName);
};

#endif //PROJECT_GRAPH_PRINTER_H
