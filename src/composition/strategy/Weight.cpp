#include <composition/strategy/Weight.hpp>

namespace composition {

Weight::Weight(const Weights &W, const std::unordered_map<llvm::Function *, llvm::BlockFrequencyInfo *> &BFI)
    : W(W), BFI(BFI) {}

Manifest *Weight::decideCycle(std::vector<Manifest *> manifests) {
  return decide(manifests);
}

Manifest *Weight::decidePresentPreserved(std::vector<Manifest *> manifests) {
  return decide(manifests);
}

Manifest *Weight::decide(std::vector<Manifest *> manifests) {
  std::unordered_map<Manifest *, Score> scores{};

  for (auto &m : manifests) {
    scores[m] = calculateScore(*m);
  }

  std::sort(manifests.begin(), manifests.end(), [scores](Manifest *m1, Manifest *m2) {
    auto m1It = scores.at(m1);
    auto m2It = scores.at(m2);

    return m1It < m2It;
  });

  return *manifests.begin();
}

Weight::Score Weight::calculateScore(const Manifest &m) {
  return 0;
}
}