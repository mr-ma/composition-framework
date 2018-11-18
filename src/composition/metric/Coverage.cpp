#include <composition/metric/Coverage.hpp>
namespace composition::metric {

std::unordered_set<llvm::Instruction *> Coverage::ValueToInstructions(llvm::Value *v) {
  std::unordered_set<llvm::Instruction *> instructions = {};

  if (auto *F = llvm::dyn_cast<llvm::Function>(v)) {
    auto r = ValueToInstructions(F);
    instructions.insert(r.begin(), r.end());
  } else if (auto *B = llvm::dyn_cast<llvm::BasicBlock>(v)) {
    auto r = ValueToInstructions(B);
    instructions.insert(r.begin(), r.end());
  } else if (auto *I = llvm::dyn_cast<llvm::Instruction>(v)) {
    instructions.insert(I);
  }
  return instructions;
}

std::unordered_set<llvm::Instruction *> Coverage::ValueToInstructions(llvm::BasicBlock *BB) {
  std::unordered_set<llvm::Instruction *> instructions = {};
  for (auto &I : *BB) {
    instructions.insert(&I);
  }
  return instructions;

}

std::unordered_set<llvm::Instruction *> Coverage::ValueToInstructions(llvm::Function *F) {
  std::unordered_set<llvm::Instruction *> instructions = {};
  for (auto &B : *F) {
    auto r = ValueToInstructions(&B);
    instructions.insert(r.begin(), r.end());
  }
  return instructions;
}

std::unordered_set<llvm::Instruction *> Coverage::ValueToInstructions(llvm::Module *M) {
  std::unordered_set<llvm::Instruction *> instructions = {};
  for (auto &F : *M) {
    auto r = ValueToInstructions(&F);
    instructions.insert(r.begin(), r.end());
  }
  return instructions;
}

std::unordered_set<llvm::BasicBlock *> Coverage::InstructionsToBasicBlocks(std::unordered_set<llvm::Instruction *> instructions) {
  std::unordered_set<llvm::BasicBlock *> basicBlocks = {};

  for (auto *I : instructions) {
    if (I->getParent() == nullptr) {
      continue;
    }
    basicBlocks.insert(I->getParent());
  }
  return basicBlocks;
}

std::unordered_set<llvm::Function *> Coverage::BasicBlocksToFunctions(std::unordered_set<llvm::BasicBlock *> basicBlocks) {
  std::unordered_set<llvm::Function *> functions = {};

  for (auto *B : basicBlocks) {
    if (B->getParent() == nullptr) {
      continue;
    }
    functions.insert(B->getParent());
  }
  return functions;
}

}