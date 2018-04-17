#include "constraints/AnalysisPass.hpp"

using namespace llvm;

char AnalysisPass::ID = 0;

bool AnalysisPass::doFinalization(Module &module) {
	GraphPrinter(c.getGraph()).dump_dot(dbgs());
	c.expandToInstructions();
	GraphPrinter(c.getGraph()).dump_dot(dbgs());
	c.reduceToInstructions();
	GraphPrinter(c.getGraph()).dump_dot(dbgs());
	c.SCC();
	GraphPrinter(c.getGraph()).dump_dot(dbgs());
	return false;
}

void AnalysisPass::getAnalysisUsage(AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<GraphPass>();
}

bool AnalysisPass::runOnModule(llvm::Module &M) {
	//c = getAnalysis<GraphPass>().getGraph();

	return false;
}


static RegisterPass<AnalysisPass> X("constraint-analysis", "Constraint Analysis Pass",
                                    false /* Only looks at CFG */,
                                    true /* Analysis Pass */);