#ifndef COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP

#include <cstdint>
#include <string>
#include <unordered_set>
#include <composition/Manifest.hpp>

namespace composition {

class ManifestRegistry {
public:
  static void Add(Manifest *m);

  static std::unordered_set<Manifest *> &GetAll();

  static void Remove(Manifest *m);

  static void destroy();

protected:
  static ManifestIndex index;

  static std::unordered_set<Manifest *> &RegisteredManifests();
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
