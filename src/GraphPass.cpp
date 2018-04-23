#include <constraints/AnalysisPass.hpp>

using namespace llvm;

ConflictGraph &GraphPass::getGraph() {
	return Graph;
}

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<AnalysisPass>();
}

bool GraphPass::runOnModule(llvm::Module &module) {
	GraphPrinter(Graph.getGraph()).dump_dot("graph_raw.dot");
	Graph.expandToFunctions();
	GraphPrinter(Graph.getGraph()).dump_dot("graph_expanded.dot");
	Graph.reduceToFunctions();
	GraphPrinter(Graph.getGraph()).dump_dot("graph_reduced.dot");
	Graph.SCC();
	return false;
}

char GraphPass::ID = 0;


static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);