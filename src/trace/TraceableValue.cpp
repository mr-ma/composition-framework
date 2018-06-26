#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/PassSupport.h>
#include <composition/trace/TraceableValue.hpp>
#include <composition/util/functions.hpp>

using namespace llvm;
using namespace composition;

template<typename ExtraDataT>
void TraceableValueState::Config::onRAUW(const ExtraDataT &, llvm::Value *oldValue, llvm::Value *newValue) {
	dbgs() << "RAUWed value but should be preserved\n";

	auto it = ValueNameMap.find(oldValue);
	if (it == ValueNameMap.end()) {
		dbgs() << "Could not find pass that added this value to be preserved.\n";
	} else {
		dbgs() << "Value was added by pass: " << it->second.pass << "\n";
	}
	dbgs() << "Old: ";
	oldValue->print(dbgs(), true);
	dbgs() << " New: ";
	newValue->print(dbgs(), true);
	dbgs() << "\n";
	dbgs() << "Value was changed by pass: " << getPassName() << "\n";

	it->second.callback(getPassName(), oldValue, newValue);
}

template<typename ExtraDataT>
void TraceableValueState::Config::onDelete(const ExtraDataT &, llvm::Value *oldValue) {
	dbgs() << "Deleted value but should be preserved\n";
	auto it = ValueNameMap.find(oldValue);
	if (it == ValueNameMap.end()) {
		dbgs() << "Could not find pass that added this value to be preserved.\n";
	} else {
		dbgs() << "Value was added by pass: " << it->second.pass << "\n";
	}
	dbgs() << "Old: ";
	oldValue->print(dbgs(), true);
	dbgs() << "\n";
	dbgs() << "Value was deleted by pass: " << getPassName() << "\n";

	it->second.callback(getPassName(), oldValue, nullptr);
}


void TraceableValueState::clear() {
	GlobalNumbers.clear();
	ValueNameMap.clear();
}

void TraceableValueState::erase(llvm::Value *v) {
	GlobalNumbers.erase(v);
	ValueNameMap.erase(v);
}

uint64_t TraceableValueState::getNumber(llvm::Value *v, TraceableCallbackInfo info) {
	ValueNumberMap::iterator MapIter;
	bool Inserted;
	std::tie(MapIter, Inserted) = GlobalNumbers.insert({v, NextNumber});
	if (Inserted)
		NextNumber++;

	ValueNameMap.insert({v, info});
	return MapIter->second;
}

std::map<llvm::Value *, TraceableCallbackInfo> TraceableValueState::ValueNameMap = {};