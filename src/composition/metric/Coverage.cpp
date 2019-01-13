#include <composition/metric/Coverage.hpp>
namespace composition::metric {

std::set<llvm::Instruction*> Coverage::ValuesToInstructions(std::set<llvm::Value*> values) {
  std::set<llvm::Instruction*> instructions = {};
  for (auto& v : values) {
    auto newInstructions = Coverage::ValueToInstructions(v);
    for (auto& I : newInstructions) {
      instructions.insert(I);
    }
  }
  return instructions;
}

std::set<llvm::Instruction*> Coverage::ValueToInstructions(llvm::Value* v) {
  std::set<llvm::Instruction*> instructions = {};

  if (auto* F = llvm::dyn_cast<llvm::Function>(v)) {
    auto r = ValueToInstructions(F);
    instructions.insert(r.begin(), r.end());
  } else if (auto* B = llvm::dyn_cast<llvm::BasicBlock>(v)) {
    auto r = ValueToInstructions(B);
    instructions.insert(r.begin(), r.end());
  } else if (auto* I = llvm::dyn_cast<llvm::Instruction>(v)) {
    instructions.insert(I);
  }
  return instructions;
}

std::set<llvm::Instruction*> Coverage::ValueToInstructions(llvm::BasicBlock* BB) {
  std::set<llvm::Instruction*> instructions = {};
  for (auto& I : *BB) {
    instructions.insert(&I);
  }
  return instructions;
}

std::set<llvm::Instruction*> Coverage::ValueToInstructions(llvm::Function* F) {
  std::set<llvm::Instruction*> instructions = {};
  for (auto& B : *F) {
    auto r = ValueToInstructions(&B);
    instructions.insert(r.begin(), r.end());
  }
  return instructions;
}

std::set<llvm::Instruction*> Coverage::ValueToInstructions(llvm::Module* M) {
  std::set<llvm::Instruction*> instructions = {};
  for (auto& F : *M) {
    auto r = ValueToInstructions(&F);
    instructions.insert(r.begin(), r.end());
  }
  return instructions;
}

std::set<llvm::BasicBlock*> Coverage::InstructionsToBasicBlocks(std::set<llvm::Instruction*> instructions) {
  std::set<llvm::BasicBlock*> basicBlocks = {};

  for (auto* I : instructions) {
    if (I->getParent() == nullptr) {
      continue;
    }
    basicBlocks.insert(I->getParent());
  }
  return basicBlocks;
}

std::set<llvm::Function*> Coverage::BasicBlocksToFunctions(std::set<llvm::BasicBlock*> basicBlocks) {
  std::set<llvm::Function*> functions = {};

  for (auto* B : basicBlocks) {
    if (B->getParent() == nullptr) {
      continue;
    }
    functions.insert(B->getParent());
  }
  return functions;
}

} // namespace composition::metric