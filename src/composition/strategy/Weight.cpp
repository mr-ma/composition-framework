#include <composition/strategy/Weight.hpp>

namespace composition {

Weight::Weight(const Weights &W, const std::unordered_map<llvm::Function *, llvm::BlockFrequencyInfo *> &BFI)
    : W(W), BFI(BFI) {}

std::shared_ptr<Manifest> Weight::decideCycle(std::vector<std::shared_ptr<Manifest>> manifests) {
  return decide(manifests);
}

std::shared_ptr<Manifest> Weight::decidePresentPreserved(std::vector<std::shared_ptr<Manifest>> manifests) {
  return decide(manifests);
}

std::shared_ptr<Manifest> Weight::decide(std::vector<std::shared_ptr<Manifest>> manifests) {
  std::unordered_map<std::shared_ptr<Manifest>, Score> scores{};

  for (auto &m : manifests) {
    scores[m] = calculateScore(*m);
  }

  std::sort(manifests.begin(), manifests.end(), [scores](std::shared_ptr<Manifest> m1, std::shared_ptr<Manifest> m2) {
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