#ifndef COMPOSITION_FRAMEWORK_STRATEGY_RANDOM_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_RANDOM_HPP

#include <composition/strategy/Strategy.hpp>

namespace composition {
class Random : public Strategy {
public:
  explicit Random(const std::default_random_engine &RNG);
  std::shared_ptr<Manifest> decideCycle(std::vector<std::shared_ptr<Manifest>> manifests) override;
  std::shared_ptr<Manifest> decidePresentPreserved(std::vector<std::shared_ptr<Manifest>> manifests) override;
private:
  std::default_random_engine RNG;
  std::shared_ptr<Manifest> decide(std::vector<std::shared_ptr<Manifest>> manifests);
};
}
#endif //COMPOSITION_FRAMEWORK_STRATEGY_RANDOM_HPP
