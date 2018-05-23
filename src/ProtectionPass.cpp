#include <composition/GraphPass.hpp>
#include <composition/ProtectionPass.hpp>
#include <composition/ManifestRegistry.hpp>

using namespace llvm;
using namespace composition;

void ProtectionPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<GraphPass>();
}

bool ProtectionPass::runOnModule(llvm::Module &M) {
	dbgs() << "ProtectionPass running\n";

	auto && pass = getAnalysis<GraphPass>();
	std::vector<Manifest> manifests = pass.GetManifestsInOrder();

	auto patchers = *ManifestRegistry::GetAllManifestPatchers();
	for (auto m : manifests) {
		auto p = ManifestRegistry::GetPatcher(m);
		p(m);
	}
	return false;
}

char ProtectionPass::ID = 0;
static llvm::RegisterPass<ProtectionPass> X("constraint-protection", "Constraint Protection Pass",
                                            true /* Only looks at CFG */,
                                            true /* Analysis Pass */);