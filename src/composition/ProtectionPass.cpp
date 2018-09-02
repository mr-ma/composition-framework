#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/GraphPass.hpp>
#include <composition/ProtectionPass.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>
#include <composition/metric/Stats.hpp>

using namespace llvm;
namespace composition {

static llvm::RegisterPass<ProtectionPass> X("constraint-protection", "Constraint Protection Pass", true, false);

char ProtectionPass::ID = 0;

void ProtectionPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequiredTransitive<GraphPass>();
}

bool ProtectionPass::runOnModule(llvm::Module &M) {
  dbgs() << "ProtectionPass running\n";

  auto &pass = getAnalysis<GraphPass>();
  auto manifests = pass.SortedManifests();

  dbgs() << "Got " << manifests.size() << " manifests\n";

  for (auto &m : manifests) {
    m->Redo();
  }
  //TODO create postpatching manifest order/export to json

  Stats s{};
  s.collect(&M, manifests);
  s.dump(dbgs());

  PreservedValueRegistry::Clear();
  return !manifests.empty();
}
}