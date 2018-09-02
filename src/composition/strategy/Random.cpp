#include <composition/strategy/Random.hpp>

namespace composition {
Random::Random(const std::default_random_engine &RNG) : RNG(RNG) {}

std::shared_ptr<Manifest> Random::decideCycle(std::vector<std::shared_ptr<Manifest>> manifests) {
  return decide(manifests);
}

std::shared_ptr<Manifest> Random::decidePresentPreserved(std::vector<std::shared_ptr<Manifest>> manifests) {
  return decide(manifests);
}

std::shared_ptr<Manifest> Random::decide(std::vector<std::shared_ptr<Manifest>> manifests) {
  std::shuffle(manifests.begin(), manifests.end(), RNG);
  return *manifests.begin();
}

}
