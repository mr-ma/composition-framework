#ifndef COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP
#define COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP

#include <istream>
#include <llvm/Support/raw_ostream.h>
#include <nlohmann/json.hpp>

namespace composition {
struct Weights {
  float explicitInstructionCoverage;
  float implicitInstructionCoverage;

  float basicBlockProfileFrequency;
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

void to_json(nlohmann::json & j, const Weights& w);

void from_json(const nlohmann::json& j, Weights& w);
}

#endif //COMPOSITION_FRAMEWORK_METRIC_WEIGHTS_HPP
