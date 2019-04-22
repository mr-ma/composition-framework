#ifndef COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP
#define COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP

#include <istream>
#include <llvm/Support/raw_ostream.h>
#include <nlohmann/json.hpp>

namespace composition::metric {
/**
 * A collection of weights to be used in the different strategies
 */
struct Weights {
  float explicitInstructionCoverage;
  float implicitInstructionCoverage;

  float basicBlockProfileCount;
  std::map<std::string, float> protectionCosts;

  float connectivityManifest;
  float connectivityInstructions;
  float connectivityFunctions;
  float connectivityProtections;

  Weights();

  explicit Weights(std::istream &i);

  void dump(llvm::raw_ostream &o);
};

/**
 * Converts `Weights` into JSON representation.
 * @param j IN/OUT the resulting JSON
 * @param w IN the weights
 */
void to_json(nlohmann::json &j, const Weights &w);

/**
 * Converts JSON into `Weights` representation
 * @param j IN the JSON source
 * @param w IN/OUT the resulting weights
 */
void from_json(const nlohmann::json &j, Weights &w);
} // namespace composition::metric

#endif // COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP
