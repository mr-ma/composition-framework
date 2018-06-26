#include <utility>

#include <string>
#include <composition/trace/PreservedValueRegistry.hpp>

using namespace llvm;
using namespace composition;

bool PreservedValueRegistry::Register(const std::string &name, llvm::Value *v, const PreservedCallback &callback) {
	dbgs() << "Registering preserved value: " << v->getName() << "\n";
	TraceableValues()->getNumber(v, TraceableCallbackInfo(name, callback));
	return true;
}

void PreservedValueRegistry::Clear() {
	dbgs() << "All values were correctly preserved\n";
	TraceableValues()->clear();
}

TraceableValueState *PreservedValueRegistry::TraceableValues() {
	static TraceableValueState value = {};
	return &value;
}
