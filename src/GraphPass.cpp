#include <llvm/Support/Debug.h>

#include <composition/GraphPass.hpp>
#include <composition/AnalysisPass.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/AnalysisRegistry.hpp>

using namespace llvm;
using namespace composition;

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
	AU.setPreservesAll();
	AU.addRequired<AnalysisPass>();
}

bool GraphPass::runOnModule(llvm::Module &module) {
	dbgs() << "GraphPass running\n";

	auto &pass = getAnalysis<AnalysisPass>();
	Graph = std::move(pass.getGraph());
	dbgs() << "GraphPass SCC\n";
	Graph.SCC();

	//TODO modify SCC to remove cycles according to strategy

	// Get all registered analysis passes and check if one needs postpatching
	// If a pass needs postpatching then apply topological sorting before applying the protections
	auto registered = AnalysisRegistry::GetAll();
	for (const auto &passInfo : registered) {
		if (passInfo.second) {
			//TODO topological sort graph according to strategy
			break;
		}
	}

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