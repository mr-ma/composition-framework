#ifndef COMPOSITION_FRAMEWORK_GRAPHPASS_H
#define COMPOSITION_FRAMEWORK_GRAPHPASS_H

#include <set>
#include <llvm/Pass.h>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/Manifest.hpp>

namespace composition {
class GraphPass : public llvm::ModulePass {
private:
  std::unique_ptr<ProtectionGraph> Graph{};
  std::unordered_set<llvm::Function*> sensitiveFunctions{};
public:
  static char ID;
public:
  GraphPass() : ModulePass(ID) {}

  std::vector<Manifest *> SortedManifests();

  const std::unordered_set<llvm::Function *> &getSensitiveFunctions() const;

  const ManifestDependencyMap getManifestDependencyMap() const {
    return Graph->getManifestDependencyMap();
  }

  const ManifestProtectionMap getManifestProtectionMap() const {
    return Graph->getManifestProtectionMap();
  }

  void getAnalysisUsage(llvm::AnalysisUsage &usage) const override;

  bool runOnModule(llvm::Module &module) override;

  bool doFinalization(llvm::Module &module) override;
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPHPASS_H
