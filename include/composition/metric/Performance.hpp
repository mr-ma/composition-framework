#ifndef COMPOSITION_FRAMEWORK_METRIC_PERFORMANCE_HPP
#define COMPOSITION_FRAMEWORK_METRIC_PERFORMANCE_HPP

#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/Analysis/BlockFrequencyInfo.h>

namespace composition {
// src#https://github.com/rcorcs/llvm-heat-printer
class Performance {
  static bool hasProfiling(llvm::Module &M);

  static uint64_t getBlockFreq(const llvm::BasicBlock *BB, llvm::BlockFrequencyInfo *BFI, bool useHeuristic = true);

  static uint64_t getNumOfCalls(llvm::Function &callerFunction, llvm::Function &calledFunction,
                                llvm::function_ref<llvm::BlockFrequencyInfo *(llvm::Function &)> LookupBFI);

  static uint64_t getMaxFreq(llvm::Function &F, llvm::BlockFrequencyInfo *BFI, bool useHeuristic = true);

  static uint64_t getMaxFreq(llvm::Module &M,
                             llvm::function_ref<llvm::BlockFrequencyInfo *(llvm::Function &)> LookupBFI,
                             bool useHeuristic = true);
};
}
#endif //COMPOSITION_FRAMEWORK_METRIC_PERFORMANCE_HPP
