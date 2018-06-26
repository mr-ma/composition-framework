#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/trace/TraceableValue.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>
#include <composition/graph/dot.hpp>

using namespace llvm;
using namespace composition;

char AnalysisPass::ID = 0;

void AnalysisPass::getAnalysisUsage(AnalysisUsage &AU) const {
	AU.setPreservesAll();
	dbgs() << "Called analysis usage\n";
	auto registered = AnalysisRegistry::GetAll();
	dbgs() << "Got " << std::to_string(registered.size()) << "\n";

	for (auto passInfo : registered) {
		dbgs() << "Require " << std::to_string(reinterpret_cast<uintptr_t>(passInfo.first)) << "\n";
		AU.addRequiredID(passInfo.first);
	}
}

bool AnalysisPass::runOnModule(llvm::Module &M) {
	dbgs() << "AnalysisPass running\n";

	auto manifests = *ManifestRegistry::GetAll();
	for (auto &m : manifests) {
		m.idx = Graph.addProtection(m.protection, m.from, m.fromType, m.to, m.toType);
	}

	save_graph_to_dot(Graph.getGraph(), "graph_raw.dot");
	Graph.expandToFunctions();
	save_graph_to_dot(Graph.getGraph(), "graph_expanded.dot");
	Graph.reduceToFunctions();
	save_graph_to_dot(Graph.getGraph(), "graph_reduced.dot");

	return false;
}

ProtectionGraph &AnalysisPass::getGraph() {
	return Graph;
}


static RegisterPass<AnalysisPass> X("constraint-analysis", "Constraint Analysis Pass",
                                    false /* Only looks at CFG */,
                                    true /* Analysis Pass */);