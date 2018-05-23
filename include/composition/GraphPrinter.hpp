#ifndef COMPOSITION_FRAMEWORK_GRAPH_PRINTER_H
#define COMPOSITION_FRAMEWORK_GRAPH_PRINTER_H

#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/FileSystem.h>
#include <composition/graph/graph.hpp>

namespace composition {
	class GraphPrinter {
	private:
		graph_t Graph;
	private:
		void printEdges(std::unordered_map<vd, uint> map, llvm::raw_ostream &stream);

		std::unordered_map<vd, uint> printNodes(llvm::raw_ostream &stream);

	public:
		explicit GraphPrinter(graph_t graph) {
			this->Graph = graph;
		}

		void dump_dot(llvm::raw_ostream &stream);

		void dump_dot(std::string fileName);
	};
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_PRINTER_H
