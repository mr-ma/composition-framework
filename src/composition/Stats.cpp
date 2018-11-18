#include <composition/Stats.hpp>

namespace composition {
Stats::Stats(std::istream &i) {
  nlohmann::json j;
  i >> j;
  from_json(j, *this);

}

void Stats::dump(llvm::raw_ostream &o) {
  nlohmann::json j;
  to_json(j, *this);

  o << j.dump(4) << "\n";
}

void to_json(nlohmann::json &j, const Stats &s) {
  j = nlohmann::json{
      {"stats", s.stats},
      {"proposedManifests", s.proposedManifests},
      {"actualManifests", s.actualManifests},
      {"cycles", s.cycles},
      {"conflicts", s.conflicts},
      {"vertices", s.vertices},
      {"edges", s.edges},
      {"timeGraphConstruction", s.timeGraphConstruction},
      {"timeConflictDetection", s.timeConflictDetection},
      {"timeConflictResolving", s.timeConflictResolving},
  };
}

void from_json(const nlohmann::json &j, Stats &s) {
  s.stats = j.at("stats").get<metric::Stats>();
  s.proposedManifests = j.at("proposedManifests").get<size_t>();
  s.actualManifests = j.at("actualManifests").get<size_t>();
  s.cycles = j.at("cycles").get<size_t>();
  s.conflicts = j.at("conflicts").get<size_t>();
  s.vertices = j.at("vertices").get<size_t>();
  s.edges = j.at("edges").get<size_t>();
  s.timeGraphConstruction = j.at("timeGraphConstruction").get<double>();
  s.timeConflictDetection = j.at("timeConflictDetection").get<double>();
  s.timeConflictResolving = j.at("timeConflictResolving").get<double>();
}
}