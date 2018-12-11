#include <composition/strategy/Avoidance.hpp>

namespace composition::strategy {

Avoidance::Avoidance(std::unordered_map<std::string, int> order) : order(std::move(order)) {}

Manifest* Avoidance::decideCycle(std::vector<Manifest*> manifests) { return decide(manifests); }

Manifest* Avoidance::decidePresentPreserved(std::vector<Manifest*> manifests) { return decide(manifests); }

Manifest* Avoidance::decide(std::vector<Manifest*> manifests) {
  std::sort(manifests.begin(), manifests.end(), [this](Manifest* m1, Manifest* m2) {
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
} // namespace composition::strategy