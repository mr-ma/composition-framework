#ifndef COMPOSITION_FRAMEWORK_GRAPH_PASS_H
#define COMPOSITION_FRAMEWORK_GRAPH_PASS_H

#include <llvm/Pass.h>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/Manifest.hpp>

namespace composition {
	class GraphPass : public llvm::ModulePass {
	private:
		ProtectionGraph Graph{};
	public:
		static char ID;
	public:
		GraphPass() : ModulePass(ID) {}

		std::vector<Manifest> GetManifestsInOrder();

		void getAnalysisUsage(llvm::AnalysisUsage &usage) const override;

		bool runOnModule(llvm::Module &module) override;
	};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PASS_H
