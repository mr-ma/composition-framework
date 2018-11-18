#ifndef COMPOSITION_FRAMEWORK_COMPOSITIONFRAMEWORKPASS_HPP
#define COMPOSITION_FRAMEWORK_COMPOSITIONFRAMEWORKPASS_HPP

#include <set>
#include <unordered_set>
#include <vector>
#include <string>
#include <llvm/Pass.h>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/Manifest.hpp>

namespace composition {
/**
 * The entry point of the composition framework
 */
class CompositionFrameworkPass : public llvm::ModulePass {
public:
  static char ID;
private:
  /**
   * A pointer to the protection graph
   */
  std::unique_ptr<graph::ProtectionGraph> Graph{};

  /**
   * A list of sensitive functions
   */
  std::unordered_set<llvm::Function *> sensitiveFunctions{};
public:
  CompositionFrameworkPass() : ModulePass(ID) {}

  /**
   * Sorts and retrieves manifest in reverse topological order
   * @return
   */
  std::vector<Manifest *> SortedManifests();

  /*
   * LLVM standard functions
   */
  void getAnalysisUsage(llvm::AnalysisUsage &usage) const override;
  bool doInitialization(llvm::Module &module) override;
  bool runOnModule(llvm::Module &module) override;
  bool doFinalization(llvm::Module &module) override;


  bool analysisPass(llvm::Module &M);
  bool graphPass(llvm::Module &M);
  bool protectionPass(llvm::Module &M);

  /**
   * Writes the patch information to a file
   * @param patchInfos the ordered patch information
   */
  void writePatchInfo(std::vector<std::pair<std::string, std::string>> patchInfos);
};
}
#endif //COMPOSITION_FRAMEWORK_COMPOSITIONFRAMEWORKPASS_HPP
