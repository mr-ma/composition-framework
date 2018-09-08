#ifndef COMPOSITION_FRAMEWORK_PROTECTIONPASS_HPP
#define COMPOSITION_FRAMEWORK_PROTECTIONPASS_HPP

#include "llvm/Pass.h"
namespace composition {

class ProtectionPass : public llvm::ModulePass {
public:
  static char ID;
public:
  ProtectionPass() : ModulePass(ID) {}

  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override;

  bool runOnModule(llvm::Module &M) override;
  void writeToFile(std::vector<std::pair<std::string, std::string>> patchInfos);
};
}

#endif //COMPOSITION_FRAMEWORK_PROTECTIONPASS_HPP
