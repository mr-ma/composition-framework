#ifndef COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP

#include <unordered_map>
#include <composition/strategy/Strategy.hpp>

namespace composition {
class Avoidance : public Strategy {
public:
  Avoidance(const std::unordered_map<std::string, int> &order);
  std::shared_ptr<Manifest> decideCycle(std::vector<std::shared_ptr<Manifest>> manifests) override;
  std::shared_ptr<Manifest> decidePresentPreserved(std::vector<std::shared_ptr<Manifest>> manifests) override;
private:
  std::unordered_map<std::string, int> order;
  std::shared_ptr<Manifest> decide(std::vector<std::shared_ptr<Manifest>> manifests);
};
}
#endif //COMPOSITION_FRAMEWORK_STRATEGY_AVOIDANCE_HPP
