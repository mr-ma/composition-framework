#ifndef COMPOSITION_FRAMEWORK_PRESERVEDVALUEREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_PRESERVEDVALUEREGISTRY_HPP

#include <llvm/IR/Value.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/trace/TraceableValue.hpp>

class PreservedValueRegistry {
public:
	static bool Register(llvm::Value *v) {
		llvm::dbgs() << "Registering preserved value: " << v->getName() << "\n";
		TraceableValues()->getNumber(v);
		return true;
	}

	static void Clear() {
		llvm::dbgs() << "All values were correctly preserved\n";
		TraceableValues()->clear();
	}
protected:
	static TraceableValueState *TraceableValues() {
		static TraceableValueState value = {};
		return &value;
	};
};

#endif //COMPOSITION_FRAMEWORK_PRESERVEDVALUEREGISTRY_HPP
