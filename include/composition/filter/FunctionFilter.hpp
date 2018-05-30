#ifndef SELF_CHECKSUMMING_FUNCTION_FILTER_H
#define SELF_CHECKSUMMING_FUNCTION_FILTER_H

#include <unordered_set>
#include <llvm/Pass.h>
#include <composition/filter/FunctionInfo.hpp>

namespace composition {

	class FunctionFilterPass : public llvm::ModulePass {
	public:
		static char ID;
	public:
		FunctionFilterPass() : llvm::ModulePass(ID) {}

	public:
		bool runOnModule(llvm::Module &M) override;

		void getAnalysisUsage(llvm::AnalysisUsage &AU) const override;

		FunctionInformation *getFunctionsInfo();

		void loadFile(llvm::Module &M, std::string file_name);

	private:
		FunctionInformation FunctionsInfo;
	};
}

#endif //SELF_CHECKSUMMING_FUNCTION_FILTER_H