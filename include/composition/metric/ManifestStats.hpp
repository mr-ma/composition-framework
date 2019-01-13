#ifndef COMPOSITION_METRIC_MANIFESTSSTATS_HPP
#define COMPOSITION_METRIC_MANIFESTSSTATS_HPP

#include <cmath>
#include <composition/Manifest.hpp>

namespace composition::metric {
struct ManifestStats {
  size_t explicitC{};
  size_t implicitC{};
  size_t hotness{};
  size_t hotnessProtectee{};

  double normalizedExplicitC{};
  double normalizedImplicitC{};
  double normalizedHotness{};
  double normalizedHotnessProtectee{};

  void normalize(std::pair<size_t, size_t> explicitC, std::pair<size_t, size_t> implicitC,
                 std::pair<size_t, size_t> hotness, std::pair<size_t, size_t> hotnessProtectee) {
    normalizeExplicitC(explicitC.first, explicitC.second);
    normalizeImplicitC(implicitC.first, implicitC.second);
    normalizeHotness(hotness.first, hotness.second);
    normalizeHotnessProtectee(hotnessProtectee.first, hotnessProtectee.second);
  }

  void normalizeExplicitC(size_t minExplicitC, size_t maxExplicitC) {
    normalizedExplicitC =
        static_cast<double>(explicitC - minExplicitC) / static_cast<double>(maxExplicitC - minExplicitC);
    if (std::isnan(normalizedExplicitC)) {
      normalizedExplicitC = 0;
    }
  }

  void normalizeImplicitC(size_t minImplicitC, size_t maxImplicitC) {
    normalizedImplicitC =
        static_cast<double>(implicitC - minImplicitC) / static_cast<double>(maxImplicitC - minImplicitC);
    if (std::isnan(normalizedImplicitC)) {
      normalizedImplicitC = 0;
    }
  }

  void normalizeHotness(size_t minHotness, size_t maxHotness) {
    normalizedHotness = static_cast<double>(hotness - minHotness) / static_cast<double>(maxHotness - minHotness);
    if (std::isnan(normalizedHotness)) {
      normalizedHotness = 0;
    }
  }

  void normalizeHotnessProtectee(size_t minHotnessProtectee, size_t maxHotnessProtectee) {
    normalizedHotnessProtectee = static_cast<double>(hotnessProtectee - minHotnessProtectee) /
                                 static_cast<double>(maxHotnessProtectee - minHotnessProtectee);
    if (std::isnan(normalizedHotnessProtectee)) {
      normalizedHotnessProtectee = 0;
    }
  }
};
} // namespace composition::metric
#endif // COMPOSITION_METRIC_MANIFESTSSTATS_HPP