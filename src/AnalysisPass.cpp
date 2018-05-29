#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/trace/TraceableValue.hpp>
#include <composition/PreservedValueRegistry.hpp>

using namespace llvm;
using namespace composition;

char AnalysisPass::ID = 0;

void AnalysisPass::getAnalysisUsage(AnalysisUsage &AU) const {
	AU.setPreservesAll();
	dbgs() << "Called analysis usage\n";
	auto registered = AnalysisRegistry::GetAll();
	dbgs() << "Got " << std::to_string(registered->size()) << "\n";

	for (auto PassID : *registered) {
		dbgs() << "Require " << std::to_string(reinterpret_cast<uintptr_t>(PassID)) << "\n";
		AU.addRequiredID(PassID);
	}
}

bool AnalysisPass::runOnModule(llvm::Module &M) {
	//c = getAnalysis<GraphPass>().getGraph();
	dbgs() << "AnalysisPass running\n";

	auto manifests = *ManifestRegistry::GetAll();
	for (Manifest m : manifests) {
		m.idx = Graph.addProtection(m.protection, m.from, m.fromType, m.to, m.toType);
	}

	GraphPrinter(Graph.getGraph()).dump_dot("graph_raw.dot");
	Graph.expandToFunctions();
	GraphPrinter(Graph.getGraph()).dump_dot("graph_expanded.dot");
	Graph.reduceToFunctions();
	GraphPrinter(Graph.getGraph()).dump_dot("graph_reduced.dot");

	return false;
}

ProtectionGraph &AnalysisPass::getGraph() {
	return Graph;
}


static RegisterPass<AnalysisPass> X("constraint-analysis", "Constraint Analysis Pass",
                                    false /* Only looks at CFG */,
                                    true /* Analysis Pass */);