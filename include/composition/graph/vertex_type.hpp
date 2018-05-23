#ifndef COMPOSITION_FRAMEWORK_VERTEX_HPP
#define COMPOSITION_FRAMEWORK_VERTEX_HPP

#include <llvm/IR/Function.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <boost/graph/properties.hpp>

namespace composition {
	enum vertex_type {
		FUNCTION,
		BASICBLOCK,
		INSTRUCTION,
		VALUE
	};

	template<typename T>
	struct TypeToNodeType;
	template<>
	struct TypeToNodeType<llvm::Function *> { static constexpr vertex_type value = FUNCTION; };
	template<>
	struct TypeToNodeType<llvm::BasicBlock *> { static constexpr vertex_type value = BASICBLOCK; };
	template<>
	struct TypeToNodeType<llvm::Instruction *> { static constexpr vertex_type value = INSTRUCTION; };
	template<>
	struct TypeToNodeType<llvm::Value *> { static constexpr vertex_type value = VALUE; };
}

namespace boost {
	enum vertex_type_t { vertex_type };
	BOOST_INSTALL_PROPERTY(vertex, type);
}

#endif //COMPOSITION_FRAMEWORK_VERTEX_HPP
