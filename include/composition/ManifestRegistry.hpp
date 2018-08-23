#ifndef COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <composition/Manifest.hpp>

namespace composition {

class ManifestRegistry {
public:
  static void Add(std::shared_ptr<Manifest> m);

  static std::unordered_map<ManifestIndex, std::shared_ptr<Manifest>> &GetAll();

  static void Remove(ManifestIndex idx);

  static void destroy();

protected:
  static ManifestIndex index;

  static std::unordered_map<ManifestIndex, std::shared_ptr<Manifest>> &RegisteredManifests();;
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
