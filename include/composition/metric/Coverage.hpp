#ifndef COMPOSITION_FRAMEWORK_COVERAGE_HPP
#define COMPOSITION_FRAMEWORK_COVERAGE_HPP

#include <unordered_set>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>

namespace composition {
class Coverage {
public:
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::Value *v);
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::BasicBlock *v);
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::Function *v);
  static std::unordered_set<llvm::BasicBlock *> InstructionsToBasicBlocks(std::unordered_set<llvm::Instruction *> instructions);
  static std::unordered_set<llvm::Function *> BasicBlocksToFunctions(std::unordered_set<llvm::BasicBlock *> basicBlocks);
};
}

#endif //COMPOSITION_FRAMEWORK_COVERAGE_HPP
