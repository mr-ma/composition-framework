#ifndef COMPOSITION_FRAMEWORK_TRACEABLEVALUE_HPP
#define COMPOSITION_FRAMEWORK_TRACEABLEVALUE_HPP

#include <map>
#include <llvm/IR/Value.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/ValueMap.h>

class TraceableValueState {
	struct Config : llvm::ValueMapConfig<llvm::Value *> {
		enum { FollowRAUW = true };

		template<typename ExtraDataT>
		static void onRAUW(const ExtraDataT &data, llvm::Value *oldValue, llvm::Value *newValue);

		template<typename ExtraDataT>
		static void onDelete(const ExtraDataT &data, llvm::Value *oldValue);
	};

	// Each GlobalValue is mapped to an identifier. The Config ensures when RAUW
	// occurs, the mapping is changed.
	using ValueNumberMap = llvm::ValueMap<llvm::Value *, uint64_t, Config>;
	ValueNumberMap GlobalNumbers;

	static std::map<llvm::Value *, std::string> ValueNameMap;

	// The next unused serial number to assign to a global.
	uint64_t NextNumber = 0;

public:
	TraceableValueState() = default;

	uint64_t getNumber(std::string name, llvm::Value *v);

	void erase(llvm::Value *v);

	void clear();
};


#endif //COMPOSITION_FRAMEWORK_TRACEABLEVALUE_HPP
