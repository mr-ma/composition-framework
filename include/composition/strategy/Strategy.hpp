#ifndef COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP

#include <composition/Manifest.hpp>

namespace composition {
class Strategy {
public:
  virtual std::shared_ptr<Manifest> decideCycle(std::vector<std::shared_ptr<Manifest>> manifests) = 0;
  virtual std::shared_ptr<Manifest> decidePresentPreserved(std::vector<std::shared_ptr<Manifest>> manifests) = 0;
};
}

#endif //COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP
