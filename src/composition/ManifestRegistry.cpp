#include <llvm/Support/Debug.h>
#include <composition/ManifestRegistry.hpp>

namespace composition {
ManifestIndex ManifestRegistry::index = 0;

void ManifestRegistry::Remove(Manifest* m) {
  auto &manifests = RegisteredManifests();
  auto it = manifests.find(m);
  if (it != manifests.end()) {
    llvm::dbgs() << "Undoing manifest...\n";
    m->dump();
    m->Undo();
    manifests.erase(it);
  }
}

std::unordered_set<Manifest*> &ManifestRegistry::GetAll() {
  return RegisteredManifests();
}

void ManifestRegistry::Add(Manifest* m) {
  m->index = index++;
  RegisteredManifests().insert(m);
}

void ManifestRegistry::destroy() {
  RegisteredManifests().clear();
}

std::unordered_set<Manifest*> &ManifestRegistry::RegisteredManifests() {
  static std::unordered_set<Manifest*> value = {};
  return value;
}

}