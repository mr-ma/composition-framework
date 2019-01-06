#ifndef COMPOSITION_FRAMEWORK_MANIFEST_HPP
#define COMPOSITION_FRAMEWORK_MANIFEST_HPP
#include <composition/graph/constraint/constraint.hpp>
#include <composition/metric/Coverage.hpp>
#include <composition/support/ManifestValueHandle.hpp>
#include <cstdint>
#include <functional>
#include <llvm/IR/Value.h>
#include <unordered_set>
#include <utility>
#include <ostream>

namespace composition {

// fwd declaration to define PatchFunction
class Manifest;

/**
 * PatchFunction is the "Redo" function. Called if the manifest stays in the program.
 */
using PatchFunction = std::function<void(const Manifest&)>;

// ManifestIndex type. 
enum class manifest_idx_t : uint64_t;
manifest_idx_t& operator++(manifest_idx_t& i);
manifest_idx_t operator++(manifest_idx_t& i, int);
std::ostream& operator<<(std::ostream& out, const manifest_idx_t& i);
llvm::raw_ostream& operator<<(llvm::raw_ostream& out, const manifest_idx_t& i);

/**
 * The class `Manifest` describes one instance of a protection and groups all relevant information to apply (redo) and
 * remove (undo) the manifests.
 */
class Manifest {
public:
  /**
   * Unique index of a manifest
   */
  manifest_idx_t index{};
  /**
   * Name of the manifest (e.g. short pass name)
   */
  std::string name;
  /**
   * The value that receives protection
   */
  support::ManifestValueHandle protectee;
  /**
   * The function that is called if the manifest stays in the program
   */
  PatchFunction patchFunction;
  /**
   * The vector of constraints associated with this manifest
   */
  std::vector<std::shared_ptr<graph::constraint::Constraint>> constraints;
  /**
   * Does this manifest require binary-level modifications?
   */
  bool postPatching;
  /**
   * If postPatching is true, what must be written to the patching manifest
   */
  std::string patchInfo;

private:
  /**
   * A vector of values that must be removed if the manifest is removed
   */
  std::vector<support::ManifestValueHandle> undoValues{};

public:
  Manifest(std::string name, llvm::Value* protectee, PatchFunction patchFunction,
           std::vector<std::shared_ptr<graph::constraint::Constraint>> constraints = {}, bool postPatching = false,
           std::set<llvm::Value*> undoValues = {}, std::string patchInfo = {});

  Manifest() = delete;

  Manifest(Manifest const&) = delete;

  Manifest(Manifest&&) = default;

  virtual ~Manifest() = default;

  Manifest& operator=(Manifest const&) = delete;

  Manifest& operator=(Manifest&&) = default;

  bool operator==(const Manifest& other) const;

  bool operator<(const Manifest& other) const;

  virtual std::unordered_set<llvm::Instruction*> Coverage() const;

  virtual std::unordered_set<llvm::Value*> UndoValues() const;

  /**
   * Called if the manifest stays in the program
   */
  virtual void Redo() const;

  /**
   * Called if the manifest is removed. Must cleanup all created values.
   */
  virtual void Undo();

  /**
   * Dumps information to dbgs()
   */
  virtual void dump();

  /**
   * Removes invalid values from the manifest
   * @return true if all constraints are still valid, false otherwise
   */
  bool Clean();
};

/**
 * Custom Hash Functor that computes the hash of the manifest
 */
class ManifestHasher {
  size_t operator()(const Manifest& m) const { 
    using T = typename std::underlying_type<composition::manifest_idx_t>::type;
    return std::hash<T>()(static_cast<T>(m.index)); 
   }
};

/**
 * Custom comparator that compares the ids of the manifests.
 */
class ManifestComparator {
  bool operator()(const Manifest& m1, const Manifest& m2) const { return m1.index == m2.index; }
};

} // namespace composition

namespace std {
// Hash definition such that Manifest can be used with sets and maps.
template <> struct hash<composition::Manifest> {
  /**
   * The actual hash function for a manifest
   * @param m the manifest
   * @return the `index` of the manifest
   */
  size_t operator()(const composition::Manifest& m) const { 
    using T = typename std::underlying_type<composition::manifest_idx_t>::type;
    return std::hash<T>()(static_cast<T>(m.index)); 
  }
};
} // namespace std

#endif // COMPOSITION_FRAMEWORK_MANIFEST_HPP
