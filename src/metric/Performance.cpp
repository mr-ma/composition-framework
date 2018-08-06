#include <composition/metric/Performance.hpp>
#include <llvm/IR/Instructions.h>

// src#https://github.com/rcorcs/llvm-heat-printer

using namespace composition;
using namespace llvm;

bool Performance::hasProfiling(Module &M) {
  for (auto &F : M) {
    for (auto &BB : F) {
      auto *TI = BB.getTerminator();
      if (TI == nullptr)
        continue;
      if (TI->getMetadata(LLVMContext::MD_prof) != nullptr)
        return true;
    }
  }
  return false;
}

uint64_t Performance::getBlockFreq(const BasicBlock *BB, BlockFrequencyInfo *BFI, bool useHeuristic) {
  uint64_t freqVal = 0;
  if (!useHeuristic) {
    auto freq = BFI->getBlockProfileCount(BB);
    if (freq.hasValue())
      freqVal = freq.getValue();
  } else {
    freqVal = BFI->getBlockFreq(BB).getFrequency();
  }
  return freqVal;
}

uint64_t Performance::getNumOfCalls(Function &callerFunction,
                                    Function &calledFunction,
                                    function_ref<BlockFrequencyInfo *(Function &)> LookupBFI) {
  auto *BFI = LookupBFI(callerFunction);
  uint64_t counter = 0;
  for (auto &BB : callerFunction) {
    uint64_t freq = getBlockFreq(&BB, BFI);
    for (auto &I : BB) {
      if (auto *Call = dyn_cast<CallInst>(&I)) {
        if (Call->getCalledFunction() == (&calledFunction))
          counter += freq;
      }
    }
  }
  return counter;
}

uint64_t Performance::getMaxFreq(Function &F, BlockFrequencyInfo *BFI, bool useHeuristic) {
  uint64_t maxFreq = 0;
  for (auto &BB : F) {
    uint64_t freqVal = getBlockFreq(&BB, BFI, useHeuristic);
    if (freqVal >= maxFreq)
      maxFreq = freqVal;
  }
  return maxFreq;
}

uint64_t Performance::getMaxFreq(Module &M,
                                 function_ref<BlockFrequencyInfo *(Function &)> LookupBFI,
                                 bool useHeuristic) {
  uint64_t maxFreq = 0;
  for (auto &F : M) {
    if (F.isDeclaration())
      continue;
    uint64_t localMaxFreq = getMaxFreq(F, LookupBFI(F), useHeuristic);
    if (localMaxFreq >= maxFreq)
      maxFreq = localMaxFreq;
  }
  return maxFreq;
}
