#ifndef COMPOSITION_FRAMEWORK_STRATEGY_WEIGHT_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_WEIGHT_HPP

#include <llvm/IR/Function.h>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <composition/metric/Weights.hpp>
#include <composition/strategy/Strategy.hpp>

namespace composition {
class Weight : public Strategy {
public:
  Weight(const Weights &W, const std::unordered_map<llvm::Function *, llvm::BlockFrequencyInfo *> &BFI);
  Manifest *decideCycle(std::vector<Manifest *> manifests) override;
  Manifest *decidePresentPreserved(std::vector<Manifest *> manifests) override;
private:
  Weights W;
  std::unordered_map<llvm::Function *, llvm::BlockFrequencyInfo *> BFI;
  Manifest *decide(std::vector<Manifest *> manifests);

  using Score = unsigned int;
  Score calculateScore(const Manifest &m);
};
}
#endif //COMPOSITION_FRAMEWORK_STRATEGY_WEIGHT_HPP
