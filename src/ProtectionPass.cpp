#include <llvm/Support/Debug.h>

#include <composition/GraphPass.hpp>
#include <composition/ProtectionPass.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>

using namespace llvm;
namespace composition {

void ProtectionPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequired<GraphPass>();
}

bool ProtectionPass::runOnModule(llvm::Module &M) {
  dbgs() << "ProtectionPass running\n";

  auto &pass = getAnalysis<GraphPass>();
  std::unordered_map<ManifestIndex, Manifest> manifests = pass.GetManifestsInOrder();

  for (const auto &m : manifests) {
    m.second.patchFunction(m.second);
  }

  PreservedValueRegistry::Clear();
  return !manifests.empty();
}

char ProtectionPass::ID = 0;
static llvm::RegisterPass<ProtectionPass> X("constraint-protection", "Constraint Protection Pass",
                                            true /* Only looks at CFG */,
                                            true /* Analysis Pass */);
}