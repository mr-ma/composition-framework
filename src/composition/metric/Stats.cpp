#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/metric/Stats.hpp>

namespace composition {
void to_json(nlohmann::json &j, const Stats &s) {
  j = nlohmann::json{
      {"numberOfManifests", s.numberOfManifests},
      {"numberOfAllInstructions", s.numberOfAllInstructions},
      {"numberOfProtectedFunctions", s.numberOfProtectedFunctions},
      {"numberOfProtectedInstructions", s.numberOfProtectedInstructions},
      {"numberOfProtectedDistinctInstructions", s.numberOfProtectedDistinctInstructions},
      {"numberOfImplicitlyProtectedInstructions", s.numberOfImplicitlyProtectedInstructions},
      {"numberOfDistinctImplicitlyProtectedInstructions", s.numberOfDistinctImplicitlyProtectedInstructions},
      {"numberOfProtectedInstructionsByType", s.numberOfProtectedInstructionsByType},
      {"numberOfProtectedFunctionsByType", s.numberOfProtectedFunctionsByType},
      {"instructionConnectivity", s.instructionConnectivity},
      {"functionConnectivity", s.functionConnectivity},
      {"protectionConnectivity", s.protectionConnectivity},
  };
}

void from_json(const nlohmann::json &j, Stats &s) {
  s.numberOfManifests = j.at("numberOfManifests").get<size_t>();
  s.numberOfAllInstructions = j.at("numberOfAllInstructions").get<size_t>();
  s.numberOfProtectedFunctions = j.at("numberOfProtectedFunctions").get<size_t>();
  s.numberOfProtectedInstructions = j.at("numberOfProtectedInstructions").get<size_t>();
  s.numberOfProtectedDistinctInstructions = j.at("numberOfProtectedDistinctInstructions").get<size_t>();
  s.numberOfImplicitlyProtectedInstructions = j.at("numberOfImplicitlyProtectedInstructions").get<size_t>();
  s.numberOfDistinctImplicitlyProtectedInstructions =
      j.at("numberOfDistinctImplicitlyProtectedInstructions").get<size_t>();
  s.numberOfProtectedInstructionsByType =
      j.at("numberOfProtectedInstructionsByType").get<std::unordered_map<std::string, size_t>>();
  s.numberOfProtectedFunctionsByType =
      j.at("numberOfProtectedFunctionsByType").get<std::unordered_map<std::string, size_t>>();
  s.instructionConnectivity = j.at("instructionConnectivity").get<Connectivity>();
  s.functionConnectivity = j.at("functionConnectivity").get<Connectivity>();
  s.protectionConnectivity =
      j.at("protectionConnectivity").get<std::unordered_map<std::string, std::pair<Connectivity, Connectivity>>>();
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

void Stats::collect(llvm::Value *V, std::vector<Manifest *> manifests, const ManifestDependencyMap &dep) {
  collect(Coverage::ValueToInstructions(V), std::move(manifests), dep);
}

void Stats::collect(llvm::Module *M, std::vector<Manifest *> manifests, const ManifestDependencyMap &dep) {
  collect(Coverage::ValueToInstructions(M), std::move(manifests), dep);
}

void Stats::collect(std::unordered_set<llvm::Function *> sensitiveFunctions,
                    std::vector<Manifest *> manifests,
                    const ManifestDependencyMap &dep) {
  std::unordered_set<llvm::Instruction *> instructions{};

  for (auto F : sensitiveFunctions) {
    auto result = Coverage::ValueToInstructions(F);
    instructions.insert(result.begin(), result.end());
  }

  collect(instructions, std::move(manifests), dep);
}

std::unordered_set<llvm::Instruction *> implictInstructions(Manifest *m, const ManifestDependencyMap &dep) {
  std::unordered_set<llvm::Instruction *> result{};
  std::queue<Manifest *> q{};

  for (auto[it, it_end] = dep.left.equal_range(m); it != it_end; ++it) {
    q.push(it->second);
  }
  std::unordered_set<Manifest *> seen{};
  while (!q.empty()) {
    auto next = q.front();
    seen.insert(next);
    q.pop();

    result.merge(next->Coverage());

    for (auto[it, it_end] = dep.left.equal_range(next); it != it_end; ++it) {
      if (seen.find(it->second) != seen.end()) {
        continue;
      }
      q.push(it->second);
    }
  }

  return result;
}

void Stats::collect(std::unordered_set<llvm::Instruction *> allInstructions,
                    std::vector<Manifest *> manifests,
                    const ManifestDependencyMap &dep) {
  this->numberOfManifests = manifests.size();
  this->numberOfAllInstructions = allInstructions.size();

  std::unordered_map<std::string, std::unordered_set<llvm::Instruction *>> instructionProtections{};
  std::unordered_map<std::string, std::unordered_map<llvm::Instruction *, size_t>> protectionConnectivityMap;

  for (auto &m : manifests) {
    auto manifestCoverage = m->Coverage();
    this->protectedInstructionsDistinct.insert(manifestCoverage.begin(), manifestCoverage.end());
    this->protectedInstructions[m->name].insert(manifestCoverage.begin(), manifestCoverage.end());

    auto manifestFunctions = Coverage::BasicBlocksToFunctions(Coverage::InstructionsToBasicBlocks(manifestCoverage));
    this->protectedFunctions[m->name].insert(manifestFunctions.begin(), manifestFunctions.end());

    for (auto &I : manifestCoverage) {
      instructionProtections[m->name].insert(I);
      protectionConnectivityMap[m->name][I]++;
    }
  }

  std::unordered_map<llvm::Instruction *, size_t> instructionConnectivityMap;
  for (auto &I : allInstructions) {
    instructionConnectivityMap[I] = 0;
  }
  for (auto&[p, instr] : instructionProtections) {
    for (auto I : instr) {
      instructionConnectivityMap[I]++;
    }
  }

  std::unordered_set<llvm::Instruction *> implicitlyCoveredInstructions{};
  std::unordered_map<Manifest *, std::unordered_set<llvm::Instruction *>> manifestImplicitlyCoveredInstructions{};
  for (auto &m : manifests) {
    manifestImplicitlyCoveredInstructions[m] = implictInstructions(m, dep);
  }
  for (auto &[m, instr] : manifestImplicitlyCoveredInstructions) {
    implicitlyCoveredInstructions.insert(instr.begin(), instr.end());
    this->numberOfImplicitlyProtectedInstructions += instr.size();
  }
  this->numberOfDistinctImplicitlyProtectedInstructions = implicitlyCoveredInstructions.size();

  for (const auto&[protection, instructions] : this->protectedInstructions) {
    this->numberOfProtectedInstructionsByType[protection] = instructions.size();
    this->numberOfProtectedInstructions += instructions.size();
  }
  this->numberOfProtectedDistinctInstructions = this->protectedInstructionsDistinct.size();

  for (const auto&[protection, functions] : this->protectedFunctions) {
    this->numberOfProtectedFunctionsByType[protection] = functions.size();
    this->numberOfProtectedFunctions += functions.size();
  }

  std::tie(instructionConnectivity, functionConnectivity) = instructionFunctionConnectivity(instructionConnectivityMap);

  for (const auto&[key, value] : protectionConnectivityMap) {
    protectionConnectivity.insert({key, instructionFunctionConnectivity(value)});
  }
}

std::pair<Connectivity, Connectivity>
Stats::instructionFunctionConnectivity(
    const std::unordered_map<llvm::Instruction *, size_t> &instructionConnectivityMap) {
  std::unordered_map<llvm::Function *, size_t> functionConnectivityMap;
  std::vector<size_t> connectivity{};
  connectivity.reserve(instructionConnectivityMap.size());
  for (auto &[I, c] : instructionConnectivityMap) {
    connectivity.push_back(c);

    if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
      continue;
    }
    auto *F = I->getFunction();
    auto num = functionConnectivityMap[F];
    functionConnectivityMap[F] = std::max(c, num);
  }
  auto instConnectivity = Connectivity{connectivity};

  connectivity.clear();
  connectivity.reserve(functionConnectivityMap.size());
  for (auto &[_, c] : functionConnectivityMap) {
    connectivity.push_back(c);
  }
  auto funcConnectivity = Connectivity{connectivity};

  return {instConnectivity, funcConnectivity};
}
}