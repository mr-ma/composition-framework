#include <constraints/AnalysisPass.hpp>
#include <composition/ManifestRegistry.hpp>

using namespace llvm;
using namespace composition;

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<AnalysisPass>();
}

bool GraphPass::runOnModule(llvm::Module &module) {
	dbgs() << "GraphPass running\n";

	auto && pass = getAnalysis<AnalysisPass>();
	Graph = std::move(pass.getGraph());
	dbgs() << "GraphPass SCC\n";
	Graph.SCC();
	return false;
}

char GraphPass::ID = 0;

std::vector<Manifest> GraphPass::GetManifestsInOrder() {
	return std::vector<Manifest>(*ManifestRegistry::GetAll());
}


static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);