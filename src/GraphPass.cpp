#include <llvm/Support/Debug.h>

#include <composition/GraphPass.hpp>
#include <composition/AnalysisPass.hpp>
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

	//TODO modify SCC to remove cycles according to strategy

	//TODO topological sort graph according to strategy

	//TODO create postpatching manifest order/export to json
	return false;
}

char GraphPass::ID = 0;

std::vector<Manifest> GraphPass::GetManifestsInOrder() {
	return std::vector<Manifest>(*ManifestRegistry::GetAll());
}


static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);