#include <composition/metric/Stats.hpp>
#include <lemon/connectivity.h>
#include <lemon/list_graph.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace composition::metric {
Stats::Stats(std::istream& i) {
  nlohmann::json j;
  i >> j;
  from_json(j, *this);
}

Stats::Stats(std::set<Manifest*> manifests) { setManifests(manifests); }

void Stats::setManifests(std::set<Manifest*> manifests) {
  for (auto& m : manifests) {
    MANIFESTS.insert({m->index, m});
  }
}

void Stats::dump(llvm::raw_ostream& o) {
  nlohmann::json j;
  to_json(j, *this);

  o << j.dump(4) << "\n";
}

void Stats::collect(llvm::Value* V, std::vector<Manifest*> manifests, const ManifestProtectionMap& dep) {
  collect(Coverage::ValueToInstructions(V), std::move(manifests), dep);
}

void Stats::collect(llvm::Module* M, std::vector<Manifest*> manifests, const ManifestProtectionMap& dep) {
  collect(Coverage::ValueToInstructions(M), std::move(manifests), dep);
}

void Stats::collect(std::set<llvm::Function*> sensitiveFunctions, std::vector<Manifest*> manifests,
                    const ManifestProtectionMap& dep) {
  std::set<llvm::Instruction*> instructions{};

  for (auto F : sensitiveFunctions) {
    auto result = Coverage::ValueToInstructions(F);
    instructions.insert(result.begin(), result.end());
  }

  collect(instructions, std::move(manifests), dep);
}

std::unordered_map<Manifest*, std::set<llvm::Instruction*>>
Stats::implictInstructions(const ManifestProtectionMap& dep, std::unordered_map<manifest_idx_t, Manifest*> MANIFESTS) {

  lemon::ListDigraph G{};
  lemon::ListDigraph::NodeMap<std::set<llvm::Instruction*>> coverage{G};
  lemon::ListDigraph::NodeMap<manifest_idx_t> indices{G};
  std::unordered_map<manifest_idx_t, lemon::ListDigraph::Node> nodes{};

  llvm::dbgs() << "Graph Nodes\n";
  for (auto [idx, m] : MANIFESTS) {
    auto n = G.addNode();
    coverage[n] = m->Coverage();
    indices[n] = idx;
    nodes.insert({idx, n});
  }

  llvm::dbgs() << "Graph Edges\n";
  for (auto it = dep.left.begin(), it_end = dep.left.end(); it != it_end; ++it) {
    G.addArc(nodes.at(it->first), nodes.at(it->second));
  }

  llvm::dbgs() << "Topological Sort\n";
  lemon::ListDigraph::NodeMap<int> order{G};
  lemon::topologicalSort(G, order);

  const auto topNodes = static_cast<unsigned long>(lemon::countNodes(G));
  std::vector<lemon::ListDigraph::Node> sorted(topNodes);

  for (lemon::ListDigraph::NodeIt n(G); n != lemon::INVALID; ++n) {
    sorted[order[n]] = n;
  }

  llvm::dbgs() << "SCC Graph\n";
  lemon::ListDigraph::NodeMap<int> compMap{G};
  lemon::stronglyConnectedComponents(G, compMap);

  std::unordered_map<int, std::vector<lemon::ListDigraph::Node>> components{};

  for (lemon::ListDigraph::NodeIt n(G); n != lemon::INVALID; ++n) {
    components[compMap[n]].push_back(n);
  }

  for (auto [idx, c] : components) {
    if (c.size() == 1) {
      continue;
    }

    std::set<llvm::Instruction*> componentCoverage{};
    for (auto n : c) {
      componentCoverage.insert(coverage[n].begin(), coverage[n].end());
    }

    for (auto n : c) {
      coverage[n] = componentCoverage;
    }
  }

  llvm::dbgs() << "Calc\n";
  for (auto n : sorted) {
    for (lemon::ListDigraph::InArcIt e(G, n); e != lemon::INVALID; ++e) {
      auto other = G.source(e);
      coverage[n].insert(coverage[other].begin(), coverage[other].end());
    }
  }

  llvm::dbgs() << "Map and correct data\n";
  std::unordered_map<Manifest*, std::set<llvm::Instruction*>> result{};
  for (lemon::ListDigraph::NodeIt n(G); n != lemon::INVALID; ++n) {
    std::set<llvm::Instruction*> corrected{};

    // Assumption: A manifest cannot protect itself. Thus, remove the explicit manifest coverage
    auto own = MANIFESTS.at(indices[n])->Coverage();
    std::set_difference(coverage[n].begin(), coverage[n].end(), own.begin(), own.end(),
                        std::inserter(corrected, corrected.begin()));

    result.insert({MANIFESTS.at(indices[n]), corrected});
  }

  return result;
}

void Stats::collect(std::set<llvm::Instruction*> allInstructions, std::vector<Manifest*> manifests,
                    const ManifestProtectionMap& dep) {
  this->numberOfManifests = manifests.size();
  this->numberOfAllInstructions = allInstructions.size();

  std::unordered_map<std::string, std::unordered_set<llvm::Instruction*>> instructionProtections{};
  std::unordered_map<std::string, std::unordered_map<llvm::Instruction*, size_t>> protectionConnectivityMap;

  llvm::dbgs() << "Getting Explicit Coverage\n";
  for (auto& m : manifests) {
    auto manifestCoverage = m->Coverage();
    this->protectedInstructionsDistinct.insert(manifestCoverage.begin(), manifestCoverage.end());
    this->protectedInstructions[m->name].insert(manifestCoverage.begin(), manifestCoverage.end());

    auto manifestFunctions = Coverage::BasicBlocksToFunctions(Coverage::InstructionsToBasicBlocks(manifestCoverage));
    this->protectedFunctions[m->name].insert(manifestFunctions.begin(), manifestFunctions.end());

    for (auto& I : manifestCoverage) {
      instructionProtections[m->name].insert(I);
      protectionConnectivityMap[m->name][I]++;
    }
  }

  llvm::dbgs() << "Preparing Instruction Connectivity\n";
  std::unordered_map<llvm::Instruction*, size_t> instructionConnectivityMap;
  for (auto& I : allInstructions) {
    instructionConnectivityMap[I] = 0;
  }
  for (auto& [p, instr] : instructionProtections) {
    for (auto I : instr) {
      instructionConnectivityMap[I]++;
    }
  }

  llvm::dbgs() << "Getting Implicit Coverage\n";
  std::set<llvm::Instruction*> implicitlyCoveredInstructions{};
  std::unordered_map<Manifest*, std::set<llvm::Instruction*>> manifestImplicitlyCoveredInstructions =
      implictInstructions(dep, MANIFESTS);
  llvm::dbgs() << "Done\n";

  for (auto& [m, instr] : manifestImplicitlyCoveredInstructions) {
    implicitlyCoveredInstructions.insert(instr.begin(), instr.end());
    this->numberOfImplicitlyProtectedInstructions += instr.size();
  }
  this->numberOfDistinctImplicitlyProtectedInstructions = implicitlyCoveredInstructions.size();

  llvm::dbgs() << "Getting Protection Coverage and Connectivity\n";
  for (const auto& [protection, instructions] : this->protectedInstructions) {
    this->numberOfProtectedInstructionsByType[protection] = instructions.size();
    this->numberOfProtectedInstructions += instructions.size();
  }
  this->numberOfProtectedDistinctInstructions = this->protectedInstructionsDistinct.size();

  for (const auto& [protection, functions] : this->protectedFunctions) {
    this->numberOfProtectedFunctionsByType[protection] = functions.size();
    this->numberOfProtectedFunctions += functions.size();
  }

  llvm::dbgs() << "Getting Connectivities\n";
  std::tie(instructionConnectivity, functionConnectivity) = instructionFunctionConnectivity(instructionConnectivityMap);

  std::set<llvm::BasicBlock*> blocks = Coverage::InstructionsToBasicBlocks(allInstructions);
  blockConnectivity = computeBlockConnectivity(blocks, manifests);

  for (const auto& [key, value] : protectionConnectivityMap) {
    protectionConnectivity.insert({key, instructionFunctionConnectivity(value)});
  }
}

std::pair<Connectivity, Connectivity> Stats::instructionFunctionConnectivity(
    const std::unordered_map<llvm::Instruction*, size_t>& instructionConnectivityMap) {
  std::unordered_map<llvm::Function*, size_t> functionConnectivityMap;
  std::vector<size_t> connectivity{};
  connectivity.reserve(instructionConnectivityMap.size());
  for (auto& [I, c] : instructionConnectivityMap) {
    connectivity.push_back(c);

    if (I->getParent() == nullptr || I->getParent()->getParent() == nullptr) {
      continue;
    }
    auto* F = I->getFunction();
    auto num = functionConnectivityMap[F];
    functionConnectivityMap[F] = std::max(c, num);
  }
  auto instConnectivity = Connectivity{connectivity};

  connectivity.clear();
  connectivity.reserve(functionConnectivityMap.size());
  for (auto& [_, c] : functionConnectivityMap) {
    connectivity.push_back(c);
  }
  auto funcConnectivity = Connectivity{connectivity};

  return {instConnectivity, funcConnectivity};
}

Connectivity Stats::computeBlockConnectivity(std::set<llvm::BasicBlock*> blocks, std::vector<Manifest*> manifests) {
  std::map<llvm::BasicBlock*, std::set<Manifest*>> mapping{};
  for (auto& BB : blocks) {
    mapping[BB] = {};
  }
  this->numberOfBlocks = mapping.size();

  for (auto& m : manifests) {
    auto blocks = m->BlockCoverage();
    for (auto& BB : blocks) {
      mapping[BB].insert(m);
    }
  }

  std::vector<size_t> result{};
  for (auto& [BB, mapped] : mapping) {
    result.push_back(mapped.size());
  }
  this->numberOfProtectedBlocks = result.size();

  return Connectivity{result};
}
void to_json(nlohmann::json& j, const Stats& s) {
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
      {"numberOfBlocks", s.numberOfBlocks},
      {"numberOfProtectedBlocks", s.numberOfProtectedBlocks},
      {"instructionConnectivity", s.instructionConnectivity},
      {"blockConnectivity", s.blockConnectivity},
      {"functionConnectivity", s.functionConnectivity},
      {"protectionConnectivity", s.protectionConnectivity},
  };
}

void from_json(const nlohmann::json& j, Stats& s) {
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
  s.numberOfBlocks = j.at("numberOfBlocks").get<size_t>();
  s.numberOfProtectedBlocks = j.at("numberOfProtectedBlocks").get<size_t>();
  s.instructionConnectivity = j.at("instructionConnectivity").get<Connectivity>();
  s.blockConnectivity = j.at("blockConnectivity").get<Connectivity>();
  s.functionConnectivity = j.at("functionConnectivity").get<Connectivity>();
  s.protectionConnectivity =
      j.at("protectionConnectivity").get<std::unordered_map<std::string, std::pair<Connectivity, Connectivity>>>();
}
} // namespace composition::metric