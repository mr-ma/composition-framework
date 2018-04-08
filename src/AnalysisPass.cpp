#include "constraints/AnalysisPass.hpp"

using namespace llvm;

char AnalysisPass::ID = 0;

bool AnalysisPass::doFinalization(Module &module) {
	GraphPrinter(c.getGraph()).dump_dot();
	c.expand();
	GraphPrinter(c.getGraph()).dump_dot();
	c.reduce();
	GraphPrinter(c.getGraph()).dump_dot();
	c.SCC();
	GraphPrinter(c.getGraph()).dump_dot();
	return false;
}

void AnalysisPass::getAnalysisUsage(AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<GraphPass>();
}

bool AnalysisPass::doInitialization(Module &module) {
	c = getAnalysis<GraphPass>().getGraph();
	return false;
}


static RegisterPass<AnalysisPass> X("constraint-analysis", "Constraint Analysis Pass",
                                    false /* Only looks at CFG */,
                                    true /* Analysis Pass */);