#ifndef COMPOSITION_FRAMEWORK_STATS_HPP
#define COMPOSITION_FRAMEWORK_STATS_HPP

#include <istream>
#include <llvm/Support/raw_ostream.h>
#include <nlohmann/json.hpp>
#include <composition/metric/Stats.hpp>

namespace composition {

class CompositionStats {
public:
  Stats stats{};
  size_t proposedManifests{};
  size_t actualManifests{};
  double timeGraphConstruction{};
  double timeCycleDetection{};
  double timeConflictResolving{};

  CompositionStats() = default;

  explicit CompositionStats(std::istream &i);

  void dump(llvm::raw_ostream &o);
};

void to_json(nlohmann::json &j, const CompositionStats &s);

void from_json(const nlohmann::json &j, CompositionStats &s);
}
#endif //COMPOSITION_FRAMEWORK_STATS_HPP
