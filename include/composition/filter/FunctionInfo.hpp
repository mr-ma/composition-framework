#ifndef COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
#define COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP

#include <unordered_set>
#include <llvm/IR/Function.h>

namespace composition {
	class FunctionInformation {
	public:
		void add(llvm::Function *F) noexcept;

		bool has(llvm::Function *F) const noexcept;

		const std::unordered_set<llvm::Function *> &all() const noexcept;

		size_t size() const noexcept;

	private:
		std::unordered_set<llvm::Function *> Functions{};
	};
}

#endif //COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
