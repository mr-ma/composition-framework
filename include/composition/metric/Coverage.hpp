#ifndef COMPOSITION_FRAMEWORK_METRIC_COVERAGE_HPP
#define COMPOSITION_FRAMEWORK_METRIC_COVERAGE_HPP

#include <unordered_map>
#include <unordered_set>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <boost/graph/adjacency_list.hpp>

namespace composition::metric {
/**
 * A collection of helpers to calculate coverage and connectivity metrics.
 */
class Coverage {
public:
  /**
   * Returns all instructions which `v` represents
   * @param v the Value
   * @return a set of instructions representing `v`
   */
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::Value *v);

  /**
   * Returns all instructions which `BB` represents
   * @param BB the BasicBlock
   * @return a set of instructions representing `BB`
   */
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::BasicBlock *BB);

  /**
   * Returns all instructions which `F` represents
   * @param F the Function
   * @return a set of instructions representing `F`
   */
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::Function *F);

  /**
   * Returns all instructions which `M` represents
   * @param M the Module
   * @return a set of instructions representing `M`
   */
  static std::unordered_set<llvm::Instruction *> ValueToInstructions(llvm::Module *M);

  /**
   * Finds all BasicBlocks used in the given instructions
   * @param instructions the instructions
   * @return a set of basicblocks
   */
  static std::unordered_set<llvm::BasicBlock *> InstructionsToBasicBlocks(std::unordered_set<llvm::Instruction *> instructions);

  /**
   * Finds all Functions used in the given basicblocks
   * @param basicBlocks the basicblocks
   * @return a set of functions
   */
  static std::unordered_set<llvm::Function *> BasicBlocksToFunctions(std::unordered_set<llvm::BasicBlock *> basicBlocks);
};
}

#endif //COMPOSITION_FRAMEWORK_METRIC_COVERAGE_HPP
