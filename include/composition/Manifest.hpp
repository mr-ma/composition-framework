#ifndef COMPOSITION_FRAMEWORK_MANIFEST_HPP
#define COMPOSITION_FRAMEWORK_MANIFEST_HPP
#include <utility>
#include <unordered_set>
#include <cstdint>
#include <llvm/IR/Value.h>
#include <composition/graph/constraint.hpp>
#include <composition/metric/Coverage.hpp>

namespace composition {

class Manifest;

typedef std::function<void(const Manifest &)> PatchFunction;
typedef unsigned long ManifestIndex;

class Manifest {
public:
  ManifestIndex idx{};
  std::string name;
  llvm::Value *protectee;
  PatchFunction patchFunction;
  std::vector<std::shared_ptr<Constraint>> constraints;
  bool postPatching;
  std::set<llvm::Value *> undoValues;
  std::set<llvm::Instruction *> guardInstructions;

  Manifest(std::string name,
           llvm::Value *protectee,
           PatchFunction patchFunction,
           std::vector<std::shared_ptr<Constraint>> constraints = {},
           bool postPatching = false,
           std::set<llvm::Value *> undoValues = {},
           std::set<llvm::Instruction *> guardInstructions = {});

  bool operator==(const Manifest &other) const;

  bool operator<(const Manifest &other) const;

  virtual std::set<llvm::Instruction *> Coverage() const;

  virtual std::set<llvm::Instruction *> GuardInstructions() const;

  virtual void Redo() const;

  virtual void Undo() const;

  virtual void dump() const;
};

}

namespace std {
template<>
struct hash<composition::Manifest> {
  size_t operator()(const composition::Manifest &pt) const {
    return pt.idx;
  }
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFEST_HPP
