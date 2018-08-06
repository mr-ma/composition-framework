#include <utility>

#ifndef COMPOSITION_FRAMEWORK_MANIFEST_HPP
#define COMPOSITION_FRAMEWORK_MANIFEST_HPP
#include <utility>
#include <unordered_set>
#include <cstdint>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/graph/vertex.hpp>
#include <composition/graph/constraint.hpp>
#include <composition/metric/Coverage.hpp>

namespace composition {

class Manifest;

typedef std::function<void(const Manifest&)> PatchFunction;

class Manifest {
public:
  unsigned long idx{};
  std::string name;
  llvm::Value *protectee;
  PatchFunction patchFunction;
  std::vector<std::shared_ptr<Constraint>> constraints;
  bool postPatching;
  std::set<llvm::Value *> undoValues;

  Manifest(std::string name,
           llvm::Value *protectee,
           PatchFunction patchFunction,
           std::vector<std::shared_ptr<Constraint>> constraints = {},
           bool postPatching = false,
           std::set<llvm::Value *> addedValues = {})
      : name(std::move(name)), protectee(protectee), patchFunction(patchFunction),
        constraints(std::move(constraints)), postPatching(postPatching), undoValues(std::move(addedValues)) {
  }

  bool operator==(const Manifest &other) const {
    return (idx == other.idx);
  }

  bool operator<(const Manifest &other) const {
    return (idx < other.idx);
  }

  virtual std::set<llvm::Instruction *> Coverage() const {
    return Coverage::ValueToInstructions(protectee);
  };

  virtual void Redo() const {
    patchFunction(*this);
  };

  virtual void Undo() const {
    //TODO strictly speaking this function only removes the Instructions and GlobalVariables.
    //TODO however, if we wanted to remove basicblocks or functions we'd need to restore the correct controlflow
    //TODO for now, assume passes only add instructions
    llvm::dbgs() << "Undoing " << undoValues.size() << " values\n";
    for (auto V : undoValues) {
      if (auto *F = llvm::dyn_cast<llvm::Function>(V)) {
        for (auto &B : *F) {
          for (auto &I : B) {
            I.eraseFromParent();
          }
        }
      } else if (auto *B = llvm::dyn_cast<llvm::BasicBlock>(V)) {
        for (auto &I : *B) {
          I.eraseFromParent();
        }
      } else if (auto *I = llvm::dyn_cast<llvm::Instruction>(V)) {
        I->eraseFromParent();
      } else if (auto *G = llvm::dyn_cast<llvm::GlobalVariable>(V)) {
        G->eraseFromParent();
      } else {
        llvm::dbgs() << "Failed to undo\n";
      }
    }
  };
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
