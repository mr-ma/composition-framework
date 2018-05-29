#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/trace/TraceableValue.hpp>
#include <llvm/PassSupport.h>

using namespace llvm;

template<typename ExtraDataT>
void TraceableValueState::Config::onRAUW(const ExtraDataT &, llvm::Value * oldValue, llvm::Value *newValue) {
	llvm::dbgs() << "RAUWed value but should be preserved\n";
}

template<typename ExtraDataT>
void TraceableValueState::Config::onDelete(const ExtraDataT &, llvm::Value *oldValue) {
	llvm::dbgs() << "Deleted value but should be preserved\n";
}


void TraceableValueState::clear() {
	GlobalNumbers.clear();
}

void TraceableValueState::erase(llvm::Value *Global) {
	GlobalNumbers.erase(Global);
}

uint64_t TraceableValueState::getNumber(llvm::Value *Global) {
	ValueNumberMap::iterator MapIter;
	bool Inserted;
	std::tie(MapIter, Inserted) = GlobalNumbers.insert({Global, NextNumber});
	if (Inserted)
		NextNumber++;
	return MapIter->second;
}
