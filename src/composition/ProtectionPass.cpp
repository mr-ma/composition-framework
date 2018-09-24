#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/raw_os_ostream.h>
#include <nlohmann/json.hpp>
#include <composition/GraphPass.hpp>
#include <composition/ProtectionPass.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/Analysis.hpp>

using namespace llvm;
namespace composition {

static llvm::RegisterPass<ProtectionPass> X("constraint-protection", "Constraint Protection Pass", true, false);

char ProtectionPass::ID = 0;

void ProtectionPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequiredTransitive<GraphPass>();

  auto registered = AnalysisRegistry::GetAll();
  for (auto passInfo : registered) {
    AU.addRequiredID(passInfo.ID);
  }
}

bool ProtectionPass::runOnModule(llvm::Module &M) {
  dbgs() << "ProtectionPass running\n";

  auto &pass = getAnalysis<GraphPass>();
  auto manifests = pass.SortedManifests();
  auto sensitiveFunctions = pass.getSensitiveFunctions();

  assert(manifests.size() == ManifestRegistry::GetAll().size());

  dbgs() << "Got " << manifests.size() << " manifests\n";
  cStats.actualManifests = manifests.size();

  std::vector<std::pair<std::string, std::string>> patchInfos{};
  size_t i = 0;
  size_t total = manifests.size();
  for (auto *m : manifests) {
    if (!m->Clean()) {
      llvm_unreachable("Manifest not clean...");
    }
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "\r";

    m->Redo();
    if (m->postPatching) {
      patchInfos.emplace_back(m->name, m->patchInfo);
    }
  }
  dbgs() << "#" << std::to_string(i) << "/" << std::to_string(total) << "\n";

  auto registered = AnalysisRegistry::GetAll();
  for (auto a : registered) {
    auto *p = getResolver()->findImplPass(a.ID);

    if (p == nullptr) {
      llvm_unreachable("Pass no longer available...");
    }
    auto *c = static_cast<composition::Pass *>(p);
    c->finalizeComposition();
  }

  writeToFile(patchInfos);

  cStats.stats.collect(sensitiveFunctions, manifests, pass.getManifestDependencyMap());
  cStats.dump(dbgs());

  if (!DumpStats.empty()) {
    auto fdStream = std::ofstream(DumpStats.getValue(), std::ofstream::out);
    if (fdStream.good()) {
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

void ProtectionPass::writeToFile(std::vector<std::pair<std::string, std::string>> patchInfos) {
  nlohmann::json j = patchInfos;
  std::ofstream file(PatchInfo.getValue());
  file << j.dump(4);
  file.close();
}
}