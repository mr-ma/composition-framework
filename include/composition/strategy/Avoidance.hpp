#ifndef COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP

#include <composition/strategy/Strategy.hpp>
#include <unordered_map>

namespace composition::strategy {
/**
 * Avoidance strategy uses a configuration file to decide which manifest must be removed.
 */
class Avoidance : public Strategy {
public:
  explicit Avoidance(std::unordered_map<std::string, int> order);
  Manifest* decideCycle(std::vector<Manifest*> manifests) override;
  Manifest* decidePresentPreserved(std::vector<Manifest*> manifests) override;

private:
  std::unordered_map<std::string, int> order;
  Manifest* decide(std::vector<Manifest*> manifests);
};
} // namespace composition::strategy
#endif // COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP
