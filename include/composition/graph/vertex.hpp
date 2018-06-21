#ifndef COMPOSITION_FRAMEWORK_VERTEX_HPP
#define COMPOSITION_FRAMEWORK_VERTEX_HPP

#include <cstdint>
#include <string>
#include <unordered_set>
#include <llvm/IR/Function.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <iostream>
#include <type_traits>

typedef uintptr_t vertex_idx_t;

enum class vertex_type {
	UNKNOWN,
	FUNCTION,
	BASICBLOCK,
	INSTRUCTION,
	VALUE
};

std::ostream &operator<<(std::ostream &os, const vertex_type &obj);

template<typename T>
struct LLVMToVertexType;
template<>
struct LLVMToVertexType<llvm::Function *> { static constexpr vertex_type value = vertex_type::FUNCTION; };
template<>
struct LLVMToVertexType<llvm::BasicBlock *> { static constexpr vertex_type value = vertex_type::BASICBLOCK; };
template<>
struct LLVMToVertexType<llvm::Instruction *> { static constexpr vertex_type value = vertex_type::INSTRUCTION; };
template<>
struct LLVMToVertexType<llvm::Value *> { static constexpr vertex_type value = vertex_type::VALUE; };

struct vertex_t {
	explicit vertex_t(
			vertex_idx_t index = 0,
			const std::string &name = "",
			vertex_type type = vertex_type::UNKNOWN,
			const std::unordered_set<std::string> &attributes = {}
	) noexcept;

	vertex_idx_t index;
	std::string name;
	vertex_type type;
	std::unordered_set<std::string> attributes;
};

std::ostream &operator<<(std::ostream &os, const vertex_t &e) noexcept;

bool operator==(const vertex_t &lhs, const vertex_t &rhs) noexcept;

bool operator!=(const vertex_t &lhs, const vertex_t &rhs) noexcept;

#endif //COMPOSITION_FRAMEWORK_VERTEX_HPP
