#ifndef COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP

#include <cstdint>
#include <string>
#include <unordered_set>
#include <composition/Manifest.hpp>

namespace composition {

/**
 * `ManifestRegistry` stores registered manifests
 */
class ManifestRegistry {
public:
  /**
   * Adds a `Manifest` to the registry.
   * @param m the pointer to the `Manifest`
   */
  static void Add(Manifest *m);

  /**
   * Retrieves and returns all registered manifests.
   * @return an `unordered_set` of `Manifest` pointers
   */
  static std::unordered_set<Manifest *> &GetAll();

  /**
   * Removes a manifest from the registry.
   * @param m the pointer to the `Manifest`
   */
  static void Remove(Manifest *m);

  /**
   * Destroys this registry. Must be called before LLVM terminates to clear memory correctly.
   */
  static void destroy();

protected:
  /**
   * Strictly increasing counter of the next manifest index. Thus, each manifest index is unique.
   */
  static ManifestIndex index;

  //TODO: This currently initializes a static unordered_set in the function. Possibly there's a better way to do this.
  static std::unordered_set<Manifest *> &RegisteredManifests();
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
