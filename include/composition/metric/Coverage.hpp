#ifndef COMPOSITION_FRAMEWORK_COVERAGE_HPP
#define COMPOSITION_FRAMEWORK_COVERAGE_HPP

#include <set>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>

namespace composition {
class Coverage {
public:
  static std::set<llvm::Instruction *> ValueToInstructions(llvm::Value *v);
  static std::set<llvm::Instruction *> ValueToInstructions(llvm::BasicBlock *v);
  static std::set<llvm::Instruction *> ValueToInstructions(llvm::Function *v);
  static std::set<llvm::BasicBlock *> InstructionsToBasicBlocks(std::set<llvm::Instruction *> instructions);
  static std::set<llvm::Function *> BasicBlocksToFunctions(std::set<llvm::BasicBlock *> basicBlocks);
};
}

#endif //COMPOSITION_FRAMEWORK_COVERAGE_HPP
