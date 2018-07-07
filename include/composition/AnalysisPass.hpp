#ifndef COMPOSITION_FRAMEWORK_ANALYSISPASS_H
#define COMPOSITION_FRAMEWORK_ANALYSISPASS_H

#include <llvm/Pass.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <composition/graph/ProtectionGraph.hpp>
#include <composition/GraphPass.hpp>

namespace composition {
class AnalysisPass : public llvm::ModulePass {
private:
  ProtectionGraph Graph{};
public:
  static char ID;
public:
  bool doInitialization(llvm::Module &module) override;

  AnalysisPass() : ModulePass(ID) {}

  ProtectionGraph &getGraph();

  bool runOnModule(llvm::Module &M) override;

  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override;
};
}
#endif //COMPOSITION_FRAMEWORK_ANALYSISPASS_H
