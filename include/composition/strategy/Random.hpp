#ifndef COMPOSITION_FRAMEWORK_STRATEGY_RANDOM_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_RANDOM_HPP

#include <composition/strategy/Strategy.hpp>
#include <random>

namespace composition::strategy {
/**
 * Random strategy decides randomly which Manifest is removed
 */
class Random : public Strategy {
public:
  explicit Random(const std::default_random_engine& RNG);
  Manifest* decideCycle(std::vector<Manifest*> manifests) override;
  Manifest* decidePresentPreserved(std::vector<Manifest*> manifests) override;

private:
  std::default_random_engine RNG;
  Manifest* decide(std::vector<Manifest*> manifests);
};
} // namespace composition::strategy
#endif // COMPOSITION_FRAMEWORK_STRATEGY_RANDOM_HPP
