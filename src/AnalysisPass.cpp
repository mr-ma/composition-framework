#include "constraints/AnalysisPass.hpp"
#include "composition/AnalysisRegistry.hpp"

using namespace llvm;

char AnalysisPass::ID = 0;

void AnalysisPass::getAnalysisUsage(AnalysisUsage &AU) const {
	AU.setPreservesAll();
	dbgs() << "Called analysis usage\n";
	auto registered = AnalysisRegistry::GetAll();
	dbgs() << "Got " << std::to_string(registered->size()) << "\n";

	for(auto PassID : *registered) {
		dbgs() << "Require " << std::to_string(reinterpret_cast<uintptr_t>(PassID)) << "\n";
		AU.addRequiredID(PassID);
	}
}

bool AnalysisPass::runOnModule(llvm::Module &M) {
	//c = getAnalysis<GraphPass>().getGraph();
	return false;
}


static RegisterPass<AnalysisPass> X("constraint-analysis", "Constraint Analysis Pass",
                                    false /* Only looks at CFG */,
                                    true /* Analysis Pass */);