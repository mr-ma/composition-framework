#ifndef COMPOSITION_FRAMEWORK_METRIC_STATS_HPP
#define COMPOSITION_FRAMEWORK_METRIC_STATS_HPP

#include <cstddef>
#include <istream>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>
#include <nlohmann/json.hpp>
#include <composition/Manifest.hpp>

namespace composition {
class Stats {
public:
  size_t numberOfManifests{};
  size_t numberOfAllInstructions{};
  size_t numberOfProtectedFunctions{};
  size_t numberOfProtectedInstructions{};
  size_t numberOfProtectedDistinctInstructions{};
  std::map<std::string, size_t> numberOfProtectedInstructionsByType{};
  std::map<std::string, size_t> numberOfProtectedFunctionsByType{};
  double avgInstructionConnectivity{};
  double avgFunctionConnectivity{};
  double stdInstructionConnectivity{};
  double stdFunctionConnectivity{};

  Stats() = default;

  explicit Stats(std::istream &i);

  void dump(llvm::raw_ostream &o);

  void collect(llvm::Module *M, std::vector<std::shared_ptr<Manifest>> manifests);

  void collect(llvm::Value *V, std::vector<std::shared_ptr<Manifest>> manifests);

  void collect(std::set<llvm::Instruction *> allInstructions, std::vector<std::shared_ptr<Manifest>> manifests);
private:
  std::set<llvm::Instruction *> protectedInstructionsDistinct{};
  std::map<std::string, std::set<llvm::Instruction *>> protectedInstructions{};
  std::map<std::string, std::set<llvm::Function *>> protectedFunctions{};

  std::pair<double, double> calculateConnectivity(std::vector<size_t> v);
};
void to_json(nlohmann::json &j, const Stats &s);

void from_json(const nlohmann::json &j, Stats &s);
}

#endif //COMPOSITION_FRAMEWORK_METRIC_STATS_HPP
