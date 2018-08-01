#ifndef COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP

#include <cstdint>
#include <string>
#include <unordered_map>
#include <composition/Manifest.hpp>

namespace composition {

typedef unsigned long ManifestIndex;
class ManifestRegistry {
public:
  static void Add(Manifest m);

  static std::unordered_map<ManifestIndex, Manifest> &GetAll();;

  static void Remove(ManifestIndex idx);

protected:
  static ManifestIndex index;

  static std::unordered_map<ManifestIndex, Manifest> &RegisteredManifests() {
    static std::unordered_map<ManifestIndex, Manifest> value = {};
    return value;
  };
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
