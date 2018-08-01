#include <composition/ManifestRegistry.hpp>
namespace composition {
ManifestIndex ManifestRegistry::index = 0;

void ManifestRegistry::Remove(ManifestIndex idx) {
  std::unordered_map<ManifestIndex, Manifest> manifests = RegisteredManifests();
  auto it = manifests.find(idx);
  if (it != manifests.end()) {
    it->second.Undo();
    manifests.erase(it);
  }
}

std::unordered_map<ManifestIndex, Manifest> &ManifestRegistry::GetAll() {
  return RegisteredManifests();
}

void ManifestRegistry::Add(Manifest m) {
  m.idx = index++;
  RegisteredManifests().insert({m.idx, std::move(m)});
}

}