#include <composition/Stats.hpp>

namespace composition {
void to_json(nlohmann::json &j, const CompositionStats &s) {
  j = nlohmann::json{
      {"stats", s.stats},
      {"proposedManifests", s.proposedManifests},
      {"actualManifests", s.actualManifests},
      {"timeGraphConstruction", s.timeGraphConstruction},
      {"timeCycleDetection", s.timeCycleDetection},
      {"timeConflictResolving", s.timeConflictResolving},
  };
}

void from_json(const nlohmann::json &j, CompositionStats &s) {
  s.stats = j.at("stats").get<Stats>();
  s.proposedManifests = j.at("proposedManifests").get<size_t>();
  s.actualManifests = j.at("actualManifests").get<size_t>();
  s.timeGraphConstruction = j.at("timeGraphConstruction").get<double>();
  s.timeCycleDetection = j.at("timeCycleDetection").get<double>();
  s.timeConflictResolving = j.at("timeConflictResolving").get<double>();
}

CompositionStats::CompositionStats(std::istream &i) {
  nlohmann::json j;
  i >> j;
  from_json(j, *this);

}

void CompositionStats::dump(llvm::raw_ostream &o) {
  nlohmann::json j;
  to_json(j, *this);

  o << j.dump(4) << "\n";
}
}