#include <composition/Manifest.hpp>
#include <composition/graph/constraint/dependency.hpp>
#include <composition/graph/constraint/present.hpp>
#include <composition/graph/constraint/preserved.hpp>
#include <composition/graph/vertex.hpp>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/Instruction.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <memory>
#include <ostream>
#include <sstream>
#include <utility>

namespace composition {
using composition::graph::constraint::Dependency;
using composition::graph::constraint::Present;
using composition::graph::constraint::Preserved;
using composition::metric::Coverage;
using llvm::dbgs;
using llvm::dyn_cast;
using llvm::isa;
using llvm::UndefValue;

manifest_idx_t& operator++(manifest_idx_t& i) {
  using T = typename std::underlying_type<manifest_idx_t>::type;
  auto val = static_cast<T>(i);
  i = manifest_idx_t(++val);
  return i;
}

const manifest_idx_t operator++(manifest_idx_t& i, int) {
  manifest_idx_t res(i);
  ++i;
  return res;
}

std::ostream& operator<<(std::ostream& out, const manifest_idx_t& i) {
  using T = typename std::underlying_type<manifest_idx_t>::type;
  out << static_cast<T>(i);
  return out;
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& out, const manifest_idx_t& i) {
  using T = typename std::underlying_type<manifest_idx_t>::type;
  out << static_cast<T>(i);
  return out;
}

void Manifest::Undo() {
  dbgs() << "Undoing " << undoValues.size() << " values\n";
  for (const auto& V : undoValues) {
    if (llvm::isa<llvm::Constant>(&*V)) {
      // Constants may not be deleted!
      continue;
    }
    dbgs() << *V << "\n";
    llvm::Value* v_copy = (&*V);
    if (!V->use_empty()) {
      V->replaceAllUsesWith(UndefValue::get(V->getType()));
    }

    if (llvm::isa<llvm::UndefValue>(v_copy)) {
      // UndefValues are constants and may not be deleted!
      continue;
    }

    if (auto* F = llvm::dyn_cast<llvm::Function>(v_copy)) {
      for (auto& B : *F) {
        for (auto& I : B) {
          I.replaceAllUsesWith(UndefValue::get(I.getType()));
          I.eraseFromParent();
        }
        B.replaceAllUsesWith(UndefValue::get(B.getType()));
        B.eraseFromParent();
      }
      F->eraseFromParent();
    } else if (auto* B = llvm::dyn_cast<llvm::BasicBlock>(v_copy)) {
      for (auto& I : *B) {
        I.eraseFromParent();
      }
    } else if (auto* I = llvm::dyn_cast<llvm::Instruction>(v_copy)) {
      I->eraseFromParent();
    } else if (auto* G = llvm::dyn_cast<llvm::GlobalVariable>(v_copy)) {
      G->eraseFromParent();
    } else {
      dbgs() << "Failed to undo\n";
      dbgs() << v_copy->getValueID() << "\n";
    }
  }
}

void Manifest::Redo() const { patchFunction(*this); }

std::set<llvm::Instruction*> Manifest::Coverage() {
  if (protectee.pointsToAliveValue()) {
    return Coverage::ValueToInstructions(&*protectee);
  }
  return {};
}

std::set<llvm::BasicBlock*> Manifest::BlockCoverage() {
  if (blockProtectee.pointsToAliveValue()) {
    return Coverage::InstructionsToBasicBlocks(Coverage::ValueToInstructions(&*blockProtectee));
  }
  return {};
}

std::set<llvm::Value*> Manifest::UndoValues() const {
  std::set<llvm::Value*> undos{};
  for (const auto& g : undoValues) {
    undos.insert(&*g);
  }
  return undos;
}

bool Manifest::operator<(const Manifest& other) const { return (index < other.index); }

bool Manifest::operator==(const Manifest& other) const { return (index == other.index); }

Manifest::Manifest(std::string name, llvm::Value* protectee, llvm::Value* blockProtectee, PatchFunction patchFunction,
                   std::vector<std::shared_ptr<graph::constraint::Constraint>> constraints, bool postPatching,
                   std::set<llvm::Value*> undoValues, std::string patchInfo)
    : name(std::move(name)), patchFunction(std::move(patchFunction)), constraints(std::move(constraints)),
      postPatching(postPatching), patchInfo(std::move(patchInfo)) {

  this->protectee = support::ManifestValueHandle(protectee);
  this->blockProtectee = support::ManifestValueHandle(blockProtectee);
  for (auto u : undoValues) {
    this->undoValues.emplace_back(u);
  }
}

std::string valueToName(llvm::Value* v) {
  if (isa<llvm::Function>(v)) {
    return v->getName();
  }
  std::stringstream s{};
  s << reinterpret_cast<uintptr_t>(v);
  return s.str();
}

void Manifest::dump() {
  Clean();

  if (protectee.pointsToAliveValue()) {
    dbgs() << "Manifest " << index << " (" << name << ") protecting " << valueToName(&*protectee) << ":\n";
  } else {
    dbgs() << "Manifest " << index << " (" << name << "): \n";
  }
  if (constraints.empty()) {
    return;
  }
  dbgs() << "\tConstraints: \n";

  for (auto& constraint : constraints) {
    dbgs() << "\t\t" << constraint->getInfo() << " ";
    if (auto d = dyn_cast<Dependency>(constraint.get())) {
      dbgs() << "Dependency between " << valueToName(d->getFrom()) << " and " << valueToName(d->getTo());
    } else if (auto present = dyn_cast<Present>(constraint.get())) {
      dbgs() << "Present of " << valueToName(present->getTarget());
    } else if (auto preserved = dyn_cast<Preserved>(constraint.get())) {
      dbgs() << "Preserved of " << valueToName(preserved->getTarget());

    } else {
      dbgs() << "Type unknown";
    }
    dbgs() << "\n";
  }
  dbgs() <<"\tCovered Instructions:  \n";
  auto coverage = this->Coverage();
  for(auto inst: coverage){
      dbgs()<<"\t\t";
      inst->dump();
  }
  dbgs()<<"\n";
}

void Manifest::Clean() {
  for (auto it = undoValues.begin(); it != undoValues.end();) {
    if (!it->pointsToAliveValue()) {
      it = undoValues.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = constraints.begin(); it != constraints.end();) {
    if (!(*it)->isValid()) {
      it = constraints.erase(it);
    } else {
      auto* dep = llvm::dyn_cast<Dependency>(it->get());
      if (dep != nullptr && dep->getFrom() == dep->getTo()) {
        it = constraints.erase(it);
      } else {
        ++it;
      }
    }
  }
}
} // namespace composition
