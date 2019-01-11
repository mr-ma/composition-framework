#include <composition/trace/PreservedValueRegistry.hpp>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <string>
#include <utility>

namespace composition::trace {
using llvm::BasicBlock;
using llvm::dbgs;
using llvm::dyn_cast;
using llvm::Function;
using llvm::Instruction;
using llvm::LLVMContext;
using llvm::MDNode;
using llvm::MDString;

TraceableCallbackInfo PreservedValueRegistry::Register(const std::string& name, llvm::Value* v,
                                                       const PreservedCallback& callback) {
  return Register(name, v, nullptr, callback);
}

TraceableCallbackInfo PreservedValueRegistry::Register(const std::string& name, llvm::Value* v,
                                                       const PresentCallback& callback) {
  return Register(name, v, callback, nullptr);
}

TraceableCallbackInfo PreservedValueRegistry::Register(const std::string& name, llvm::Value* v,
                                                       const PresentCallback& presentCallback,
                                                       const PreservedCallback& preservedCallback) {
  if (presentCallback) {
    if (auto* I = dyn_cast<Instruction>(v)) {
      LLVMContext& C = I->getContext();
      MDNode* N = MDNode::get(C, MDString::get(C, "nosub"));
      I->setMetadata("llvm.obfuscator.nosub", N);
    }
  }

  if (preservedCallback) {
    if (auto* I = dyn_cast<Instruction>(v)) {
      LLVMContext& C = I->getContext();
      MDNode* N = MDNode::get(C, MDString::get(C, "nosub"));
      I->setMetadata("llvm.obfuscator.nosub", N);
    } else if (auto* B = dyn_cast<BasicBlock>(v)) {
      for (auto& I : *B) {
        LLVMContext& C = I.getContext();
        MDNode* N = MDNode::get(C, MDString::get(C, "nosub"));
        I.setMetadata("llvm.obfuscator.nosub", N);
      }
    } else if (auto* F = dyn_cast<Function>(v)) {
      LLVMContext& C = F->getContext();
      MDNode* N = MDNode::get(C, MDString::get(C, "nosub.nofla"));
      F->setMetadata("llvm.obfuscator.nofla", N);
      F->setMetadata("llvm.obfuscator.nosub", N);
      F->setMetadata("llvm.obfuscator.nobcf", N);
    }
  }

  auto info = TraceableCallbackInfo(name, presentCallback, preservedCallback);
  TraceableValues().getNumber(v, info);
  return info;
}

void PreservedValueRegistry::Clear() {
  dbgs() << "All values were correctly preserved\n";
  TraceableValues().clear();
}

TraceableValueState& PreservedValueRegistry::TraceableValues() {
  static TraceableValueState value = {};
  return value;
}
} // namespace composition::trace