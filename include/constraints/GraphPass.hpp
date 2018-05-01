#ifndef PROJECT_CONSTRAINT_GRAPH_PASS_H
#define PROJECT_CONSTRAINT_GRAPH_PASS_H

#include "llvm/Pass.h"
#include "constraints/ConflictGraph.hpp"

class GraphPass : public llvm::ModulePass {
private:
	ConflictGraph Graph{};
public:
	static char ID;
public:
	GraphPass() : ModulePass(ID) {}

	std::vector<composition::Manifest> GetManifestsInOrder();

	void getAnalysisUsage(llvm::AnalysisUsage &usage) const override;

	bool runOnModule(llvm::Module &module) override;
};


#endif //PROJECT_CONSTRAINT_GRAPH_PASS_H
