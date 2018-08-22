#include <composition/metric/Weights.hpp>

namespace composition {
void to_json(nlohmann::json &j, const Weights &w) {
  j = nlohmann::json{{"coverage", w.coverage},
                     {"explicitInstructionCoverage", w.explicitInstructionCoverage},
                     {"implicitInstructionCoverage", w.implicitInstructionCoverage},

                     {"performance", w.performance},
                     {"basicBlockProfileCount", w.basicBlockProfileCount},
                     {"basicBlockProfileFrequency", w.basicBlockProfileFrequency},

                     {"resilience", w.resilience},
                     {"connectivityManifest", w.connectivityManifest},
                     {"connectivityInstructions", w.connectivityInstructions},
                     {"connectivityFunctions", w.connectivityFunctions},
                     {"connectivityProtections", w.connectivityProtections}
  };
}

void from_json(const nlohmann::json &j, Weights &w) {
  w.coverage = j.at("coverage").get<float>();
  w.explicitInstructionCoverage = j.at("explicitInstructionCoverage").get<float>();
  w.implicitInstructionCoverage = j.at("implicitInstructionCoverage").get<float>();

  w.performance = j.at("performance").get<float>();
  w.basicBlockProfileCount = j.at("basicBlockProfileCount").get<float>();
  w.basicBlockProfileFrequency = j.at("basicBlockProfileFrequency").get<float>();

  w.resilience = j.at("resilience").get<float>();
  w.connectivityManifest = j.at("connectivityManifest").get<float>();
  w.connectivityInstructions = j.at("connectivityInstructions").get<float>();
  w.connectivityFunctions = j.at("connectivityFunctions").get<float>();
  w.connectivityProtections = j.at("connectivityProtections").get<float>();
}
}