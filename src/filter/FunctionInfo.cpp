#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/filter/FunctionInfo.hpp>

using namespace llvm;
using namespace composition;

void FunctionInformation::add(llvm::Function *F) noexcept {
	llvm::dbgs() << "FunctionInfo. Adding function:" << F->getName() << "\n";
	Functions.insert(F);
}

bool FunctionInformation::has(llvm::Function *F) const noexcept {
	if (Functions.size() == 0 || !F) {
		return false;
	}
	return Functions.find(F) != Functions.end();
}

const std::unordered_set<llvm::Function *> &FunctionInformation::all() const noexcept {
	return Functions;
}

size_t FunctionInformation::size() const noexcept {
	return Functions.size();
}

