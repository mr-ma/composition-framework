#ifndef COMPOSITION_FRAMEWORK_STRATEGY_WEIGHT_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_WEIGHT_HPP

#include <composition/metric/Weights.hpp>
#include <composition/strategy/Strategy.hpp>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <llvm/IR/Function.h>

namespace composition::strategy {
/**
 * Weight strategy uses weighted protection graph metrics to calculate a score. Then it removes the manifest with the
 * lowest score.
 */
class Weight : public Strategy {
public:
  Weight(metric::Weights W, std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*> BFI);
  Manifest* decideCycle(std::vector<Manifest*> manifests) override;
  Manifest* decidePresentPreserved(std::vector<Manifest*> manifests) override;

private:
  metric::Weights W;
  std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*> BFI;
  Manifest* decide(std::vector<Manifest*> manifests);

  using Score = float;
  Score calculateScore(const Manifest& m);
};
} // namespace composition::strategy
#endif // COMPOSITION_FRAMEWORK_STRATEGY_WEIGHT_HPP
