#include <composition/ManifestRegistry.hpp>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_os_ostream.h>

namespace composition {
manifest_idx_t ManifestRegistry::index = manifest_idx_t(0);

void ManifestRegistry::Remove(Manifest* m) {
  auto& manifests = RegisteredManifests();

  if (manifests.find(m) != manifests.end()) {
    llvm::dbgs() << "Undoing manifest...\n";
    m->dump();
    m->Undo();
    manifests.erase(m);
  }
}

std::set<Manifest*>& ManifestRegistry::GetAll() { return RegisteredManifests(); }

void ManifestRegistry::Add(Manifest* m) {
  m->index = index++;
  RegisteredManifests().insert(m);
}

void ManifestRegistry::destroy() {
  for (auto* m : RegisteredManifests()) {
    delete m;
  }
  RegisteredManifests().clear();
}

std::set<Manifest*>& ManifestRegistry::RegisteredManifests() {
  static std::set<Manifest*> value = {};
  return value;
}

} // namespace composition