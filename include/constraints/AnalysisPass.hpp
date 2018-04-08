#ifndef PROJECT_ANALYSISPASS_H
#define PROJECT_ANALYSISPASS_H

#include "llvm/Pass.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Module.h"
#include "constraints/ConflictGraph.hpp"
#include "constraints/GraphPass.hpp"
#include "constraints/GraphPrinter.hpp"

class AnalysisPass : public llvm::ModulePass {
private:
	ConflictGraph c{};
public:
	static char ID;
public:
	AnalysisPass() : ModulePass(ID) {}

	bool doInitialization(llvm::Module &module) override;

	bool doFinalization(llvm::Module &module) override;

	bool runOnModule(llvm::Module &M) override {
		return false;
	}

	void getAnalysisUsage(llvm::AnalysisUsage &usage) const override;
};

#endif //PROJECT_ANALYSISPASS_H
