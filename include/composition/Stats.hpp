#ifndef COMPOSITION_FRAMEWORK_STATS_HPP
#define COMPOSITION_FRAMEWORK_STATS_HPP

#include <composition/metric/Stats.hpp>
#include <istream>
#include <llvm/Support/raw_ostream.h>
#include <nlohmann/json.hpp>

namespace composition {

class Stats {
public:
  metric::Stats stats{};
  size_t proposedManifests{};
  size_t actualManifests{};
  size_t cycles{};
  size_t conflicts{};
  size_t edges{};
  size_t vertices{};
  double timeGraphConstruction{};
  double timeConflictDetection{};
  double timeConflictResolving{};

  Stats() = default;

  explicit Stats(std::istream& i);

  void dump(llvm::raw_ostream& o);
};

/**
 * Converts `Stats` into JSON representation.
 * @param j IN/OUT the resulting JSON
 * @param w IN the stats
 */
void to_json(nlohmann::json& j, const Stats& s);

/**
 * Converts JSON into `Stats` representation
 * @param j IN the JSON source
 * @param w IN/OUT the resulting stats
 */
void from_json(const nlohmann::json& j, Stats& s);
} // namespace composition
#endif // COMPOSITION_FRAMEWORK_STATS_HPP
