#ifndef COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP
#define COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP

#include <nlohmann/json.hpp>

namespace composition {
struct Weights {
  float coverage;
  float explicitInstructionCoverage;
  float implicitInstructionCoverage;

  float performance;
  float basicBlockProfileFrequency;
  float basicBlockProfileCount;

  float resilience;
  float connectivityManifest;
  float connectivityInstructions;
  float connectivityFunctions;
  float connectivityProtections;

  Weights();
};

void to_json(nlohmann::json & j, const Weights& w);

void from_json(const nlohmann::json& j, Weights& w);
}

#endif //COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP
