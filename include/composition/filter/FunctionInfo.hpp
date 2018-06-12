#ifndef COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
#define COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP

#include <unordered_set>
#include <llvm/IR/Function.h>
#include <composition/filter/FilterInfo.hpp>

namespace composition {
	class FunctionInformation : public FilterInfo<llvm::Function *> {
	};
}

#endif //COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
