#include <constraints/GraphPass.hpp>
#include "constraints/ProtectionPass.hpp"

void ProtectionPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<GraphPass>();
}

bool ProtectionPass::runOnModule(llvm::Module &M) {
	return false;
}

char ProtectionPass::ID = 0;
static llvm::RegisterPass<ProtectionPass> X("constraint-protection", "Constraint Protection Pass",
                                            true /* Only looks at CFG */,
                                            true /* Analysis Pass */);