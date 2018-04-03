#include "constraints/GraphPrinter.h"
#include "llvm/Pass.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Module.h"

using namespace llvm;


namespace {

	struct ConstraintAnalysisPass : public FunctionPass {
		static char ID;
		ConflictGraph c{};

		ConstraintAnalysisPass() : FunctionPass(ID) {}

		bool doInitialization(Module &module) override;

		bool doFinalization(Module &module) override;

		bool runOnFunction(Function &F) override {
			llvm::BasicBlock *b_saved = nullptr;
			uint bb = 0;
			for (auto &B : F) {
				B.setName(std::to_string(bb));
				c.addHierarchy(&F, &B);
				for (auto &I : B) {
					c.addHierarchy(&B, &I);
				}
				b_saved = &B;
				bb++;
			}

			for (auto &B2 : F) {
				if (b_saved != &B2) {
					c.addProtection("cfi", b_saved, &B2);
					break;
				}
			}

			return false;
		}
	};
}

char ConstraintAnalysisPass::ID = 0;

bool ConstraintAnalysisPass::doInitialization(Module &module) {
	return Pass::doInitialization(module);
}

bool ConstraintAnalysisPass::doFinalization(Module &module) {
	GraphPrinter(c.getGraph()).dump_dot();
	c.expand();
	GraphPrinter(c.getGraph()).dump_dot();
	c.reduce();
	GraphPrinter(c.getGraph()).dump_dot();
	return Pass::doFinalization(module);
}


static RegisterPass<ConstraintAnalysisPass> X("constraint-analysis", "Constraint Analysis Pass",
                                              false /* Only looks at CFG */,
                                              true /* Analysis Pass */);

// Automatically enable the pass.
// http://adriansampson.net/blog/clangpass.html
static void registerConstraintAnalysisPass(const PassManagerBuilder &, legacy::PassManagerBase &PM) {
	PM.add(new ConstraintAnalysisPass());
}

static RegisterStandardPasses RegisterMyPass(PassManagerBuilder::EP_EarlyAsPossible, registerConstraintAnalysisPass);
