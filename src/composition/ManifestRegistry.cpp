#include <composition/ManifestRegistry.hpp>
namespace composition {
ManifestIndex ManifestRegistry::index = 0;

void ManifestRegistry::Remove(std::shared_ptr<Manifest> m) {
  auto& manifests = RegisteredManifests();
  auto it = manifests.find(m);
  if (it != manifests.end()) {
    m->Undo();
    manifests.erase(it);
  }
}

std::set<std::shared_ptr<Manifest>> &ManifestRegistry::GetAll() {
  return RegisteredManifests();
}

void ManifestRegistry::Add(std::shared_ptr<Manifest> m) {
  m->index = index++;
  RegisteredManifests().insert(std::move(m));
}

void ManifestRegistry::destroy() {
  RegisteredManifests().clear();
}

std::set<std::shared_ptr<Manifest>> &ManifestRegistry::RegisteredManifests() {
  static std::set<std::shared_ptr<Manifest>> value = {};
  return value;
}

}