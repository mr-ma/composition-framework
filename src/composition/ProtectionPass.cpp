#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/raw_os_ostream.h>
#include <composition/GraphPass.hpp>
#include <composition/ProtectionPass.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>

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
  cStats.actualManifests = manifests.size();

  for (auto &m : manifests) {
    m->Redo();
  }
  //TODO create postpatching manifest order/export to json
  cStats.stats.collect(&M, manifests);
  cStats.dump(dbgs());

  if(!DumpStats.empty()) {
    auto fdStream = std::ofstream(DumpStats.getValue(), std::ofstream::out);
    if(fdStream.good()) {
      dbgs() << "Dumping stats to file: " << DumpStats.getValue() << "\n";
      auto stream = raw_os_ostream(fdStream);
      cStats.dump(stream);
    } else {
      dbgs() << "Could not dump stats\n";
    }
    fdStream.close();
  }

  PreservedValueRegistry::Clear();
  return !manifests.empty();
}
}