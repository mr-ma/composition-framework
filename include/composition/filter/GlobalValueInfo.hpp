#ifndef COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
#define COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP

#include <unordered_set>
#include <llvm/IR/Function.h>

class GlobalValueInfo {
public:
	void add(llvm::Value *v) noexcept;

	bool has(llvm::Value *v) const noexcept;

	const std::unordered_set<llvm::Value *> &all() const noexcept;

	size_t size() const noexcept;

private:
	std::unordered_set<llvm::Value *> Values{};
};

#endif //COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
