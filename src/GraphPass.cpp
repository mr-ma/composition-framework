#include "constraints/GraphPass.hpp"

const ConflictGraph &GraphPass::getGraph() const {
	return Graph;
}

char GraphPass::ID = 0;


static llvm::RegisterPass<GraphPass> X("constraint-graph-pass", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);