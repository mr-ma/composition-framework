#include <composition/strategy/Random.hpp>

namespace composition::strategy {
Random::Random(const std::default_random_engine &RNG) : RNG(RNG) {}

Manifest *Random::decideCycle(std::vector<Manifest *> manifests) {
  return decide(manifests);
}

Manifest *Random::decidePresentPreserved(std::vector<Manifest *> manifests) {
  return decide(manifests);
}

Manifest *Random::decide(std::vector<Manifest *> manifests) {
  std::shuffle(manifests.begin(), manifests.end(), RNG);
  return *manifests.begin();
}

}
