#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/Manifest.hpp>
#include <composition/graph/vertex.hpp>

using namespace llvm;

namespace composition {
void Manifest::Undo() const {
  //TODO strictly speaking this function only removes the Instructions and GlobalVariables.
  //TODO however, if we wanted to remove basicblocks or functions we'd need to restore the correct controlflow
  //TODO for now, assume passes only add instructions
  dbgs() << "Undoing " << undoValues.size() << " values\n";
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
      dbgs() << "Failed to undo\n";
    }
  }
}

void Manifest::Redo() const {
  patchFunction(*this);
}

std::set<llvm::Instruction *> Manifest::Coverage() const {
  return Coverage::ValueToInstructions(protectee);
}

bool Manifest::operator<(const Manifest &other) const {
  return (idx < other.idx);
}

bool Manifest::operator==(const Manifest &other) const {
  return (idx == other.idx);
}

Manifest::Manifest(std::string name,
                   llvm::Value *protectee,
                   PatchFunction patchFunction,
                   std::vector<std::shared_ptr<Constraint>> constraints,
                   bool postPatching,
                   std::set<llvm::Value *> addedValues)
    : name(std::move(name)), protectee(protectee), patchFunction(std::move(patchFunction)),
      constraints(std::move(constraints)), postPatching(postPatching), undoValues(std::move(addedValues)) {
}

std::string valueToName(llvm::Value* v) {
  if(isa<Function>(v)) {
    return v->getName();
  }
  return std::to_string(v->getValueID());
}

void Manifest::dump() const {
  dbgs() << "Manifest " << idx << " (" << name << ") protecting " << valueToName(protectee) << ":\n";
  if(constraints.empty()) {
    return;
  }
  dbgs() << "\tConstraints: \n";
  for (const auto &c : constraints) {
    dbgs() << "\t\t" << c->getInfo() << " ";
    if (auto d = dyn_cast<Dependency>(c.get())) {
      dbgs() << "Dependency between " << valueToName(d->getFrom()) << " and " << valueToName(d->getTo());
    } else if (auto present = dyn_cast<Present>(c.get())) {
      dbgs() << "Present of " << valueToName(present->getTarget());
    } else if (auto preserved = dyn_cast<Preserved>(c.get())) {
      dbgs() << "Preserved of " << valueToName(preserved->getTarget());

    } else {
      dbgs() << "Type unknown";
    }
    dbgs() << "\n";
  }
}

}
