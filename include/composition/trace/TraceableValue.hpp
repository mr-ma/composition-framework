#ifndef COMPOSITION_FRAMEWORK_TRACEABLEVALUE_HPP
#define COMPOSITION_FRAMEWORK_TRACEABLEVALUE_HPP

#include <llvm/IR/Value.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/ValueMap.h>

class TraceableValueState {
	struct Config : llvm::ValueMapConfig<llvm::Value *> {
		enum { FollowRAUW = true };

		template<typename ExtraDataT>
		static void onRAUW(const ExtraDataT & /*Data*/, llvm::Value * /*Old*/, llvm::Value * /*New*/);

		template<typename ExtraDataT>
		static void onDelete(const ExtraDataT &/*Data*/, llvm::Value * /*Old*/);
	};

	// Each GlobalValue is mapped to an identifier. The Config ensures when RAUW
	// occurs, the mapping is changed.
	using ValueNumberMap = llvm::ValueMap<llvm::Value *, uint64_t, Config>;
	ValueNumberMap GlobalNumbers;

	// The next unused serial number to assign to a global.
	uint64_t NextNumber = 0;

public:
	TraceableValueState() = default;

	uint64_t getNumber(llvm::Value *Global);

	void erase(llvm::Value *Global);

	void clear();
};


#endif //COMPOSITION_FRAMEWORK_TRACEABLEVALUE_HPP
