#include "constraints/GraphPrinter.h"
#include "llvm/Pass.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Module.h"

using namespace llvm;


namespace {

	struct ConstraintAnalysisPass : public ModulePass {
		static char ID;
		ConflictGraph c{};

		ConstraintAnalysisPass() : ModulePass(ID) {}

		bool doInitialization(Module &module) override;

		bool doFinalization(Module &module) override;

		bool runOnModule(Module &M) override {
			llvm::BasicBlock *b_saved = nullptr;
			llvm::Function *f_saved = nullptr;
			uint bb = 0;
			uint ff = 4;
			for (auto &F : M) {
				if(ff == 0){
					f_saved = &F;
				}
				for (auto &B : F) {
					B.setName(std::to_string(bb));
					c.addHierarchy(&F, &B);
					for (auto &I : B) {
						c.addHierarchy(&B, &I);
					}
					b_saved = &B;
					bb++;
					ff--;
				}
			}
			ff = 3;
			for (auto &F : M) {
				if(ff == 0) {
					c.addProtection("sc", f_saved, &F);
					c.addProtection("cfi",  &F, f_saved);
					break;
				}
				ff--;

			}

			for(auto &F : M) {
				for (auto &B2 : F) {
					if (b_saved != &B2) {
						c.addProtection("cfi", b_saved, &B2);
						goto stop;
					}
				}
			}
			stop:

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
	c.SCC();
	c.removeProtection(1);
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
