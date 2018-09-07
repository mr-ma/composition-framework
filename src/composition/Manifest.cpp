#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/Constants.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/Manifest.hpp>
#include <composition/graph/vertex.hpp>

using namespace llvm;

namespace composition {
void Manifest::Undo() const {
  dbgs() << "Undoing " << undoValues.size() << " values\n";
  for (const auto &V : undoValues) {
    if (llvm::isa<llvm::Constant>(V)) {
      //Constants may not be deleted!
      continue;
    }
    dbgs() << *V << "\n";
    llvm::Value *v_copy = V;
    if (!V->use_empty()) {
      V->replaceAllUsesWith(UndefValue::get(V->getType()));
    }

    if (llvm::isa<llvm::UndefValue>(v_copy)) {
      //UndefValues are constants and may not be deleted!
      continue;
    }

    if (auto *F = llvm::dyn_cast<llvm::Function>(v_copy)) {
      for (auto &B : *F) {
        for (auto &I : B) {
          I.replaceAllUsesWith(UndefValue::get(I.getType()));
          I.eraseFromParent();
        }
        B.replaceAllUsesWith(UndefValue::get(B.getType()));
        B.eraseFromParent();
      }
      F->eraseFromParent();
    } else if (auto *B = llvm::dyn_cast<llvm::BasicBlock>(v_copy)) {
      for (auto &I : *B) {
        I.eraseFromParent();
      }
    } else if (auto *I = llvm::dyn_cast<llvm::Instruction>(v_copy)) {
      I->eraseFromParent();
    } else if (auto *G = llvm::dyn_cast<llvm::GlobalVariable>(v_copy)) {
      G->eraseFromParent();
    } else {
      dbgs() << "Failed to undo\n";
      dbgs() << v_copy->getValueID() << "\n";
    }
  }
}

void Manifest::Redo() const {
  patchFunction(*this);
}

std::unordered_set<llvm::Instruction *> Manifest::Coverage() const {
  if (protectee.pointsToAliveValue()) {
    return Coverage::ValueToInstructions(protectee);
  }
  return {};
}

std::unordered_set<llvm::Instruction *> Manifest::GuardInstructions() const {
  std::unordered_set<llvm::Instruction *> guards{};
  for (const auto &g : guardInstructions) {
    if (auto *I = llvm::dyn_cast<llvm::Instruction>(g)) {
      guards.insert(I);
    }
  }
  return guards;
}

std::unordered_set<llvm::Value *> Manifest::UndoValues() const {
  std::unordered_set<llvm::Value *> undos{};
  for (const auto &g : undoValues) {
    if (auto *I = llvm::dyn_cast<llvm::Instruction>(g)) {
      undos.insert(I);
    }
  }
  return undos;
}

bool Manifest::operator<(const Manifest &other) const {
  return (index < other.index);
}

bool Manifest::operator==(const Manifest &other) const {
  return (index == other.index);
}

Manifest::Manifest(std::string name,
                   llvm::Value *protectee,
                   PatchFunction patchFunction,
                   std::vector<std::shared_ptr<Constraint>> constraints,
                   bool postPatching,
                   std::set<llvm::Value *> undoValues,
                   std::set<llvm::Instruction *> guardInstructions)
    : name(std::move(name)), patchFunction(std::move(patchFunction)),
      constraints(std::move(constraints)), postPatching(postPatching) {

  this->protectee = WeakTrackingVH(protectee);
  for (auto u : undoValues) {
    this->undoValues.emplace_back(u);
  }
  for (auto g : guardInstructions) {
    this->guardInstructions.emplace_back(g);
  }
}

std::string valueToName(llvm::Value *v) {
  if (isa<Function>(v)) {
    return v->getName();
  }
  return std::to_string(reinterpret_cast<uintptr_t>(v));
}

void Manifest::dump() {
  if(protectee.pointsToAliveValue()) {
    dbgs() << "Manifest " << index << " (" << name << ") protecting " << valueToName(protectee) << ":\n";
  } else {
    dbgs() << "Manifest " << index << " (" << name << "): \n";
  }
  if (constraints.empty()) {
    return;
  }
  dbgs() << "\tConstraints: \n";

  for (auto it = constraints.begin(), it_end = constraints.end(); it != it_end; ++it) {
    if (!(*it)->isValid()) {
      it = constraints.erase(it);
      continue;
    }
    dbgs() << "\t\t" << (*it)->getInfo() << " ";
    if (auto d = dyn_cast<Dependency>((*it).get())) {
      dbgs() << "Dependency between " << valueToName(d->getFrom()) << " and " << valueToName(d->getTo());
    } else if (auto present = dyn_cast<Present>((*it).get())) {
      dbgs() << "Present of " << valueToName(present->getTarget());
    } else if (auto preserved = dyn_cast<Preserved>((*it).get())) {
      dbgs() << "Preserved of " << valueToName(preserved->getTarget());

    } else {
      dbgs() << "Type unknown";
    }
    dbgs() << "\n";
  }
}

void Manifest::Clean() {
  for (auto it = undoValues.begin(), it_end = undoValues.end(); it != it_end; ++it) {
    if (!it->pointsToAliveValue()) {
      it = undoValues.erase(it);
    }
  }
  for (auto it = guardInstructions.begin(), it_end = guardInstructions.end(); it != it_end; ++it) {
    if (!it->pointsToAliveValue()) {
      it = guardInstructions.erase(it);
    }
  }
}
}
