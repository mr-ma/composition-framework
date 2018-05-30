#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/filter/GlobalValueInfo.hpp>

void GlobalValueInfo::add(llvm::Value *v) noexcept {
	llvm::dbgs() << "FunctionInfo. Adding function:" << v->getName() << "\n";
	Values.insert(v);
}

bool GlobalValueInfo::has(llvm::Value *v) const noexcept {
	if (Values.size() == 0 || !v) {
		return false;
	}
	return Values.find(v) != Values.end();
}

const std::unordered_set<llvm::Value *> &GlobalValueInfo::all() const noexcept {
	return Values;
}

size_t GlobalValueInfo::size() const noexcept {
	return Values.size();
}

