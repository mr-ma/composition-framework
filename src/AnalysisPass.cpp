#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/trace/TraceableValue.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>
#include <composition/graph/dot.hpp>
#include <llvm/Analysis/CallGraph.h>

using namespace llvm;
using namespace composition;

char AnalysisPass::ID = 0;

bool AnalysisPass::doInitialization(Module &M) {
	dbgs() << "AnalysisPass loaded...\n";

	//The following code loads all the tags from the source code.
	//The tags are then applied to the function for which a tag was used.
	//The tags may be used by any pass to decide if it is e.g., a sensitive function
	//Start annotations from @ http://bholt.org/posts/llvm-quick-tricks.html
	auto global_annos = M.getNamedGlobal("llvm.global.annotations");
	if (global_annos) {
		auto a = cast<ConstantArray>(global_annos->getOperand(0));
		for (unsigned int i = 0; i < a->getNumOperands(); i++) {
			auto e = cast<ConstantStruct>(a->getOperand(i));

			if (auto fn = dyn_cast<Function>(e->getOperand(0)->getOperand(0))) {
				auto anno = cast<ConstantDataArray>(cast<GlobalVariable>(e->getOperand(1)->getOperand(0))->getOperand(0))->getAsCString();
				fn->addFnAttr(anno); // <-- add function annotation here
				dbgs() << "Sensitive function: " << fn->getName().str() << "\n";
			}
		}
	}
	//End annotations from @ http://bholt.org/posts/llvm-quick-tricks.html
	return true;
}

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


	//TODO #1 It's probably better to use a callgraph here. However, using a callgraph leads to a SIGSEGV for no reason
	//Printing and dumping of the callgraph works, but as soon as accessing it is tried the program behaves unexpectedly.
	//Therefore, for now, only add direct call edges as CFG part to the graph.
	for (auto &F : M) {
		// Look for calls by this function.
		for (auto &BB : F) {
			for (auto &I : BB) {
				Value *v = &cast<Value>(I);
				CallSite CS(v);
				if (!CS) {
					continue;
				}

				Function *Callee = CS.getCalledFunction();
				if (!Callee) {
					continue;
				}

				//Only direct calls are possible to track
				Graph.addCFG(&F, Callee);
			}
		}
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