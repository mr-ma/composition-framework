#include <composition/strategy/Avoidance.hpp>

namespace composition {

Avoidance::Avoidance(const std::unordered_map<std::string, int> &order) : order(order) {}

std::shared_ptr<Manifest> Avoidance::decideCycle(std::vector<std::shared_ptr<Manifest>> manifests) {
  return decide(manifests);
}

std::shared_ptr<Manifest> Avoidance::decidePresentPreserved(std::vector<std::shared_ptr<Manifest>> manifests) {
  return decide(manifests);
}

std::shared_ptr<Manifest> Avoidance::decide(std::vector<std::shared_ptr<Manifest>> manifests) {
  std::sort(manifests.begin(), manifests.end(), [this](std::shared_ptr<Manifest> m1, std::shared_ptr<Manifest> m2) {
    auto m1It = order.find(m1->name);
    auto m2It = order.find(m2->name);

    if (m1It == order.end()) {
      return false;
    }
    if (m2It == order.end()) {
      return true;
    }
    return *m1It < *m2It;
  });
  return *manifests.begin();
}
}