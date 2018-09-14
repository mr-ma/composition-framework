#ifndef COMPOSITION_FRAMEWORK_MANIFEST_HPP
#define COMPOSITION_FRAMEWORK_MANIFEST_HPP
#include <utility>
#include <unordered_set>
#include <cstdint>
#include <llvm/IR/Value.h>
#include <composition/graph/constraint.hpp>
#include <composition/metric/Coverage.hpp>
#include <composition/ManifestValueHandle.hpp>

namespace composition {

class Manifest;

using PatchFunction = std::function<void(const Manifest &)>;
using ManifestIndex = unsigned long;

class Manifest {
public:
  ManifestIndex index{};
  std::string name;
  ManifestValueHandle protectee;
  PatchFunction patchFunction;
  std::vector<std::shared_ptr<Constraint>> constraints;
  bool postPatching;
  std::string patchInfo;
private:
  std::vector<ManifestValueHandle> undoValues{};
  std::vector<ManifestValueHandle> guardInstructions{};
public:
  Manifest(std::string name,
           llvm::Value *protectee,
           PatchFunction patchFunction,
           std::vector<std::shared_ptr<Constraint>> constraints = {},
           bool postPatching = false,
           std::set<llvm::Value *> undoValues = {},
           std::set<llvm::Instruction *> guardInstructions = {},
           std::string patchInfo = {});

  Manifest() = delete;

  Manifest(Manifest const &) = delete;

  Manifest(Manifest &&) = default;

  virtual ~Manifest() = default;

  Manifest &operator=(Manifest const &) = delete;

  Manifest &operator=(Manifest &&) = default;

  bool operator==(const Manifest &other) const;

  bool operator<(const Manifest &other) const;

  virtual std::unordered_set<llvm::Instruction *> Coverage() const;

  virtual std::unordered_set<llvm::Instruction *> GuardInstructions() const;

  virtual std::unordered_set<llvm::Value *> UndoValues() const;

  virtual void Redo() const;

  virtual void Undo() const;

  virtual void dump();

  bool Clean();
};

}

namespace std {
template<>
struct hash<composition::Manifest> {
  size_t operator()(const composition::Manifest &pt) const {
    return pt.index;
  }
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFEST_HPP
