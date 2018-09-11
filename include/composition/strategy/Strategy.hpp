#ifndef COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP

#include <composition/Manifest.hpp>

namespace composition {
class Strategy {
public:
  virtual ~Strategy() = default;

  virtual Manifest* decideCycle(std::vector<Manifest*> manifests) = 0;
  virtual Manifest* decidePresentPreserved(std::vector<Manifest*> manifests) = 0;
};
}

#endif //COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP
