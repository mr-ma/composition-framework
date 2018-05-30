#include <string>
#include <composition/trace/PreservedValueRegistry.hpp>

using namespace llvm;

bool PreservedValueRegistry::Register(std::string name, llvm::Value *v) {
	dbgs() << "Registering preserved value: " << v->getName() << "\n";
	TraceableValues()->getNumber(name, v);
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
