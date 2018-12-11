#ifndef COMPOSITION_FRAMEWORK_METRIC_PERFORMANCE_HPP
#define COMPOSITION_FRAMEWORK_METRIC_PERFORMANCE_HPP

#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>

namespace composition::metric {
// @src: https://github.com/rcorcs/llvm-heat-printer

/**
 * A collection of performance checking helpers
 */
class Performance {
public:
  /**
   * Checks if profiling information for the module `M` exist.
   * @param M the Module
   * @return true if profiling information exist, false otherwise.
   */
  static bool hasProfiling(llvm::Module& M);

  /**
   * Returns the block frequency
   * @param BB the BasicBlock
   * @param BFI the BlockFrequencyInfo
   * @param useHeuristic if false, then the exact values from the profile are used.
   * @return the block frequency
   */
  static uint64_t getBlockFreq(const llvm::BasicBlock* BB, llvm::BlockFrequencyInfo* BFI, bool useHeuristic = true);

  /**
   * Returns the maximum block frequency for the function `F`.
   * @param F the Function
   * @param BFI the BlockFrequencyInfo
   * @param useHeuristic if false, then the exact values from the profile are used.
   * @return the maximum block frequency for the function `F`.
   */
  static uint64_t getMaxFreq(llvm::Function& F, llvm::BlockFrequencyInfo* BFI, bool useHeuristic = true);

  /**
   * Returns the maximum block frequency for the module `M`.
   * @param M the Module
   * @param LookupBFI the BlockFrequency lookup structure
   * @param useHeuristic if false, then the exact values from the profile are used.
   * @return the maximum block frequency for the module `M`.
   */
  static uint64_t getMaxFreq(llvm::Module& M, llvm::function_ref<llvm::BlockFrequencyInfo*(llvm::Function&)> LookupBFI,
                             bool useHeuristic = true);
};
} // namespace composition::metric
#endif // COMPOSITION_FRAMEWORK_METRIC_PERFORMANCE_HPP
