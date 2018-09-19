#ifndef COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP

#include <unordered_map>
#include <composition/strategy/Strategy.hpp>

namespace composition {
class Avoidance : public Strategy {
public:
  explicit Avoidance(const std::unordered_map<std::string, int> &order);
  Manifest *decideCycle(std::vector<Manifest *> manifests) override;
  Manifest *decidePresentPreserved(std::vector<Manifest *> manifests) override;
private:
  std::unordered_map<std::string, int> order;
  Manifest *decide(std::vector<Manifest *> manifests);
};
}
#endif //COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP
