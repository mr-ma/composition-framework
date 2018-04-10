#ifndef PROJECT_CONSTRAINT_GRAPH_PASS_H
#define PROJECT_CONSTRAINT_GRAPH_PASS_H

#include "llvm/Pass.h"
#include "constraints/ConflictGraph.hpp"

class GraphPass : public llvm::ImmutablePass {
private:
	ConflictGraph Graph{};
public:
	static char ID;
public:
	GraphPass() : ImmutablePass(ID) {}

	ConflictGraph &getGraph();
};


#endif //PROJECT_CONSTRAINT_GRAPH_PASS_H
