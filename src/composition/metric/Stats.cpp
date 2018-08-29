#include <utility>
#include <composition/metric/Stats.hpp>

namespace composition {
void to_json(nlohmann::json &j, const Stats &s) {
  j = nlohmann::json{
      {"numberOfManifests", s.numberOfManifests},
      {"numberOfAllInstructions", s.numberOfAllInstructions},
      {"numberOfProtectedFunctions", s.numberOfProtectedFunctions},
      {"numberOfProtectedInstructions", s.numberOfProtectedInstructions},
      {"numberOfProtectedDistinctInstructions", s.numberOfProtectedDistinctInstructions},
      {"numberOfProtectedInstructionsByType", s.numberOfProtectedInstructionsByType},
      {"numberOfProtectedFunctionsByType", s.numberOfProtectedFunctionsByType},
      {"avgInstructionConnectivity", s.avgInstructionConnectivity},
      {"avgFunctionConnectivity", s.avgFunctionConnectivity},
      {"stdInstructionConnectivity", s.stdInstructionConnectivity},
      {"stdFunctionConnectivity", s.stdFunctionConnectivity},
  };
}

void from_json(const nlohmann::json &j, Stats &s) {
  s.numberOfManifests = j.at("numberOfManifests").get<size_t>();
  s.numberOfAllInstructions = j.at("numberOfAllInstructions").get<size_t>();
  s.numberOfProtectedFunctions = j.at("numberOfProtectedFunctions").get<size_t>();
  s.numberOfProtectedInstructions = j.at("numberOfProtectedInstructions").get<size_t>();
  s.numberOfProtectedDistinctInstructions = j.at("numberOfProtectedDistinctInstructions").get<size_t>();
  s.numberOfProtectedInstructionsByType =
      j.at("numberOfProtectedInstructionsByType").get<std::map<std::string, size_t>>();
  s.numberOfProtectedFunctionsByType = j.at("numberOfProtectedFunctionsByType").get<std::map<std::string, size_t>>();
  s.avgInstructionConnectivity = j.at("avgInstructionConnectivity").get<double>();
  s.avgFunctionConnectivity = j.at("avgFunctionConnectivity").get<double>();
  s.stdInstructionConnectivity = j.at("stdInstructionConnectivity").get<double>();
  s.stdFunctionConnectivity = j.at("stdFunctionConnectivity").get<double>();
}

void Stats::dump(llvm::raw_ostream &o) {
  nlohmann::json j;
  to_json(j, *this);

  o << j.dump(4) << "\n";
}

Stats::Stats(std::istream &i) {
  nlohmann::json j;
  i >> j;
  from_json(j, *this);
}

void Stats::collect(llvm::Value *V, std::vector<std::shared_ptr<Manifest>> manifests) {
  collect(Coverage::ValueToInstructions(V), std::move(manifests));
}

void Stats::collect(llvm::Module *M, std::vector<std::shared_ptr<Manifest>> manifests) {
  collect(Coverage::ValueToInstructions(M), std::move(manifests));
}

void Stats::collect(std::set<llvm::Instruction *> allInstructions, std::vector<std::shared_ptr<Manifest>> manifests) {
  this->numberOfManifests = manifests.size();
  this->numberOfAllInstructions = allInstructions.size();

  std::unordered_map<llvm::Instruction *, size_t> instructionConnectivity;
  for (auto &I : allInstructions) {
    instructionConnectivity[I] = 0;
  }

  for (auto &m : manifests) {
    auto manifestCoverage = m->Coverage();
    this->protectedInstructionsDistinct.insert(manifestCoverage.begin(), manifestCoverage.end());
    this->protectedInstructions[m->name].insert(manifestCoverage.begin(), manifestCoverage.end());

    auto manifestFunctions = Coverage::BasicBlocksToFunctions(Coverage::InstructionsToBasicBlocks(manifestCoverage));
    this->protectedFunctions[m->name].insert(manifestFunctions.begin(), manifestFunctions.end());

    for (auto &I : manifestCoverage) {
      instructionConnectivity[I]++;
    }
  }

  for (const auto&[protection, instructions] : this->protectedInstructions) {
    this->numberOfProtectedInstructionsByType[protection] = instructions.size();
    this->numberOfProtectedInstructions += instructions.size();
  }
  this->numberOfProtectedDistinctInstructions = this->protectedInstructionsDistinct.size();

  for (const auto&[protection, functions] : this->protectedFunctions) {
    this->numberOfProtectedFunctionsByType[protection] = functions.size();
    this->numberOfProtectedFunctions += functions.size();
  }
  std::unordered_map<llvm::Function *, size_t> functionConnectivity;
  std::vector<size_t> connectivity{};
  connectivity.reserve(instructionConnectivity.size());
  for (auto &[I, c] : instructionConnectivity) {
    connectivity.push_back(c);

    if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
      continue;
    }
    auto *F = I->getFunction();
    auto num = functionConnectivity[F];
    functionConnectivity[F] = std::max(c, num);
  }
  std::tie(avgInstructionConnectivity, stdInstructionConnectivity) = this->calculateConnectivity(connectivity);

  connectivity.clear();
  connectivity.reserve(functionConnectivity.size());
  for (auto &[_, c] : functionConnectivity) {
    connectivity.push_back(c);
  }
  std::tie(avgFunctionConnectivity, stdFunctionConnectivity) = this->calculateConnectivity(connectivity);
}

std::pair<double, double> Stats::calculateConnectivity(std::vector<size_t> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stddev = std::sqrt(sq_sum / v.size());
  return {mean, stddev};
}
}