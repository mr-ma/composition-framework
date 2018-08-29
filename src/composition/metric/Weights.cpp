#include <composition/metric/Weights.hpp>

namespace composition {
void to_json(nlohmann::json &j, const Weights &w) {
  j = nlohmann::json{
      {"explicitInstructionCoverage", w.explicitInstructionCoverage},
      {"implicitInstructionCoverage", w.implicitInstructionCoverage},

      {"basicBlockProfileFrequency", w.basicBlockProfileFrequency},
      {"basicBlockProfileCount", w.basicBlockProfileCount},
      {"protectionCosts", w.protectionCosts},

      {"connectivityManifest", w.connectivityManifest},
      {"connectivityInstructions", w.connectivityInstructions},
      {"connectivityFunctions", w.connectivityFunctions},
      {"connectivityProtections", w.connectivityProtections}
  };
}

void from_json(const nlohmann::json &j, Weights &w) {
  w.explicitInstructionCoverage = j.at("explicitInstructionCoverage").get<float>();
  w.implicitInstructionCoverage = j.at("implicitInstructionCoverage").get<float>();

  w.basicBlockProfileFrequency = j.at("basicBlockProfileFrequency").get<float>();
  w.basicBlockProfileCount = j.at("basicBlockProfileCount").get<float>();
  w.protectionCosts = j.at("protectionCosts").get<std::map<std::string, float>>();

  w.connectivityManifest = j.at("connectivityManifest").get<float>();
  w.connectivityInstructions = j.at("connectivityInstructions").get<float>();
  w.connectivityFunctions = j.at("connectivityFunctions").get<float>();
  w.connectivityProtections = j.at("connectivityProtections").get<float>();
}

Weights::Weights() {
  explicitInstructionCoverage = 1.0;
  implicitInstructionCoverage = 1.0;

  basicBlockProfileFrequency = 1.0;
  basicBlockProfileCount = 1.0;
  protectionCosts = {};

  connectivityManifest = 1.0;
  connectivityInstructions = 1.0;
  connectivityFunctions = 1.0;
  connectivityProtections = 1.0;
}

void Weights::dump(llvm::raw_ostream &o) {
  nlohmann::json j;
  to_json(j, *this);

  o << j.dump(4) << "\n";
}

Weights::Weights(std::istream &i) {
  nlohmann::json j;
  i >> j;

  from_json(j, *this);
}
}