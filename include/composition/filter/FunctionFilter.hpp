#ifndef SELF_CHECKSUMMING_FILTER_FUNCTIONFILTER_H
#define SELF_CHECKSUMMING_FILTER_FUNCTIONFILTER_H

#include <unordered_set>
#include <llvm/Pass.h>
#include <llvm/Support/CommandLine.h>
#include <composition/filter/FunctionInfo.hpp>

namespace composition {

extern llvm::cl::opt<std::string> FilterFile;

class FunctionFilterPass : public llvm::ModulePass {
public:
  static char ID;
public:
  FunctionFilterPass() : llvm::ModulePass(ID) {}

public:
  bool runOnModule(llvm::Module &M) override;

  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override;

  FunctionInformation *getFunctionsInfo();

  void loadFile(llvm::Module &M, std::string file_name);

private:
  FunctionInformation FunctionsInfo;
};
}

#endif //SELF_CHECKSUMMING_FILTER_FUNCTIONFILTER_H