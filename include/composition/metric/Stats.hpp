#ifndef COMPOSITION_FRAMEWORK_METRIC_STATS_HPP
#define COMPOSITION_FRAMEWORK_METRIC_STATS_HPP

#include <cstddef>
#include <istream>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>
#include <nlohmann/json.hpp>
#include <composition/Manifest.hpp>
#include <composition/metric/Connectivity.hpp>

namespace composition {

using ManifestProtectionMap = boost::bimaps::bimap<boost::bimaps::multiset_of<Manifest *>,
                                                   boost::bimaps::multiset_of<Manifest *>>;

class Stats {
public:
  size_t numberOfManifests{};
  size_t numberOfAllInstructions{};
  size_t numberOfProtectedFunctions{};
  size_t numberOfProtectedInstructions{};
  size_t numberOfProtectedDistinctInstructions{};
  size_t numberOfImplicitlyProtectedInstructions{};
  size_t numberOfDistinctImplicitlyProtectedInstructions{};
  std::unordered_map<std::string, size_t> numberOfProtectedInstructionsByType{};
  std::unordered_map<std::string, size_t> numberOfProtectedFunctionsByType{};
  Connectivity instructionConnectivity{};
  Connectivity functionConnectivity{};
  std::unordered_map<std::string, std::pair<Connectivity, Connectivity>> protectionConnectivity{};

  Stats() = default;

  explicit Stats(std::istream &i);

  void dump(llvm::raw_ostream &o);

  void collect(std::unordered_set<llvm::Function *> sensitiveFunctions,
               std::vector<Manifest *> manifests,
               const ManifestProtectionMap &dep);

  void collect(llvm::Module *M, std::vector<Manifest *> manifests, const ManifestProtectionMap &dep);

  void collect(llvm::Value *V, std::vector<Manifest *> manifests, const ManifestProtectionMap &dep);

  void collect(std::unordered_set<llvm::Instruction *> allInstructions,
               std::vector<Manifest *> manifests,
               const ManifestProtectionMap &dep);
private:
  std::unordered_set<llvm::Instruction *> protectedInstructionsDistinct{};
  std::map<std::string, std::unordered_set<llvm::Instruction *>> protectedInstructions{};
  std::map<std::string, std::unordered_set<llvm::Function *>> protectedFunctions{};

  std::pair<Connectivity,
            Connectivity> instructionFunctionConnectivity(const std::unordered_map<llvm::Instruction *,
                                                                                   size_t> &instructionConnectivityMap);
};
void to_json(nlohmann::json &j, const Stats &s);

void from_json(const nlohmann::json &j, Stats &s);

}

#endif //COMPOSITION_FRAMEWORK_METRIC_STATS_HPP
