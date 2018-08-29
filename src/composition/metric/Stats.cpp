#include <utility>

#include <composition/metric/Stats.hpp>

namespace composition {
void to_json(nlohmann::json &j, const Stats &s) {
  j = nlohmann::json{{"numberOfAllInstructions", s.numberOfAllInstructions},
                     {"numberOfProtectedFunctions", s.numberOfProtectedFunctions},
                     {"numberOfProtectedInstructions", s.numberOfProtectedInstructions},
                     {"numberOfProtectedDistinctInstructions", s.numberOfProtectedDistinctInstructions},
                     {"numberOfProtectedInstructionsByType", s.numberOfProtectedInstructionsByType},
                     {"numberOfProtectedFunctionsByType", s.numberOfProtectedFunctionsByType},
                     {"avgConnectivity", s.avgConnectivity},
                     {"stdConnectivity", s.stdConnectivity},
  };
}

void from_json(const nlohmann::json &j, Stats &s) {
  s.numberOfAllInstructions = j.at("numberOfAllInstructions").get<size_t>();
  s.numberOfProtectedFunctions = j.at("numberOfProtectedFunctions").get<size_t>();
  s.numberOfProtectedInstructions = j.at("numberOfProtectedInstructions").get<size_t>();
  s.numberOfProtectedDistinctInstructions = j.at("numberOfProtectedDistinctInstructions").get<size_t>();
  s.numberOfProtectedInstructionsByType =
      j.at("numberOfProtectedInstructionsByType").get<std::map<std::string, size_t>>();
  s.numberOfProtectedFunctionsByType = j.at("numberOfProtectedFunctionsByType").get<std::map<std::string, size_t>>();
  s.avgConnectivity = j.at("avgConnectivity").get<double>();
  s.stdConnectivity = j.at("stdConnectivity").get<double>();
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
  std::unordered_map<llvm::Instruction *, size_t> instructionConnectivity;

  this->numberOfAllInstructions += allInstructions.size();

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

  std::vector<size_t> connectivity{};
  connectivity.reserve(instructionConnectivity.size());
  for (auto &[_, c] : instructionConnectivity) {
    connectivity.push_back(c);
  }
  this->calculateConnectivity(connectivity);
}

void Stats::calculateConnectivity(std::vector<size_t> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / v.size());
  this->avgConnectivity = mean;
  this->stdConnectivity = stdev;
}
}