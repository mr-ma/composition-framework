#ifndef COMPOSITION_FRAMEWORK_METRIC_STATS_HPP
#define COMPOSITION_FRAMEWORK_METRIC_STATS_HPP

#include <composition/Manifest.hpp>
#include <composition/metric/Connectivity.hpp>
#include <composition/util/bimap.hpp>
#include <cstddef>
#include <istream>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <nlohmann/json.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace composition::metric {

using ManifestProtectionMap = composition::util::bimap<manifest_idx_t, manifest_idx_t>;

/**
 * A collection of Protection Graph related metrics
 */
class Stats {
public:
  size_t numberOfManifests{};
  size_t numberOfAllInstructions{};
  size_t numberOfProtectedFunctions{};
  size_t numberOfProtectedInstructions{};
  size_t numberOfProtectedDistinctInstructions{};
  size_t numberOfImplicitlyProtectedInstructions{};
  size_t numberOfDistinctImplicitlyProtectedInstructions{};
  size_t numberOfBlocks{};
  size_t numberOfProtectedBlocks{};
  std::unordered_map<std::string, size_t> numberOfProtectedInstructionsByType{};
  std::unordered_map<std::string, size_t> numberOfProtectedFunctionsByType{};
  Connectivity instructionConnectivity{};
  Connectivity blockConnectivity{};
  Connectivity functionConnectivity{};
  std::unordered_map<std::string, std::pair<Connectivity, Connectivity>> protectionConnectivity{};
  std::unordered_map<manifest_idx_t, Manifest*> MANIFESTS{};

  Stats() = default;

  explicit Stats(std::set<Manifest*> manifests);

  explicit Stats(std::istream& i);

  void dump(llvm::raw_ostream& o);

  void setManifests(std::set<Manifest*> manifests);

  void collect(std::set<llvm::Function*> sensitiveFunctions, std::vector<Manifest*> manifests,
               const ManifestProtectionMap& dep);

  void collect(llvm::Module* M, std::vector<Manifest*> manifests, const ManifestProtectionMap& dep);

  void collect(llvm::Value* V, std::vector<Manifest*> manifests, const ManifestProtectionMap& dep);

  void collect(std::set<llvm::Instruction*> allInstructions, std::vector<Manifest*> manifests,
               const ManifestProtectionMap& dep);

  std::unordered_map<Manifest*, std::unordered_set<llvm::Instruction*>>
  implictInstructions(const ManifestProtectionMap& dep, std::unordered_map<manifest_idx_t, Manifest*> MANIFESTS);
  std::vector<std::tuple<manifest_idx_t /*edge_index*/, std::pair<manifest_idx_t, manifest_idx_t> /*m1 -> m2*/,
                         unsigned long /*coverage*/>>
  implictInstructionsPerEdge(const ManifestProtectionMap& dep, std::unordered_map<manifest_idx_t, Manifest*> MANIFESTS);
private:
  std::set<llvm::Instruction*> protectedInstructionsDistinct{};
  std::map<std::string, std::set<llvm::Instruction*>> protectedInstructions{};
  std::map<std::string, std::set<llvm::Function*>> protectedFunctions{};

  std::pair<Connectivity, Connectivity>
  instructionFunctionConnectivity(const std::unordered_map<llvm::Instruction*, size_t>& instructionConnectivityMap);

  Connectivity computeBlockConnectivity(std::set<llvm::BasicBlock*> blocks, std::vector<Manifest*> manifests);
};

/**
 * Converts `Stats` into JSON representation.
 * @param j IN/OUT the resulting JSON
 * @param w IN the stats
 */
void to_json(nlohmann::json& j, const Stats& s);

/**
 * Converts JSON into `Stats` representation
 * @param j IN the JSON source
 * @param w IN/OUT the resulting stats
 */
void from_json(const nlohmann::json& j, Stats& s);

} // namespace composition::metric

#endif // COMPOSITION_FRAMEWORK_METRIC_STATS_HPP
