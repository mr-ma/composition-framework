#ifndef COMPOSITION_FRAMEWORK_COVERAGE_HPP
#define COMPOSITION_FRAMEWORK_COVERAGE_HPP

#include <set>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <boost/graph/adjacency_list.hpp>

namespace composition {
class Coverage {
public:
  static std::set<llvm::Instruction *> ValueToInstructions(llvm::Value *v);
  static std::set<llvm::Instruction *> ValueToInstructions(llvm::BasicBlock *v);
  static std::set<llvm::Instruction *> ValueToInstructions(llvm::Function *v);
  static std::set<llvm::BasicBlock *> InstructionsToBasicBlocks(std::set<llvm::Instruction *> instructions);
  static std::set<llvm::Function *> BasicBlocksToFunctions(std::set<llvm::BasicBlock *> basicBlocks);

  template<typename graph_t>
  static std::map<llvm::Instruction *, uint64_t> InstructionConnectivity(llvm::Module *M, graph_t &g) {
    std::map<llvm::Instruction *, uint64_t> result{};

    for (auto &F : *M) {
      for (auto &BB : F) {
        for (auto &I : BB) {
          result.insert({&I, 0});
        }
      }
    }

    for (auto[vi, vi_end] = boost::vertices(g); vi != vi_end; ++vi) {
      auto v = reinterpret_cast<llvm::Value *>(g[*vi].index);
      if (auto I = llvm::dyn_cast<llvm::Instruction>(v)) {
        result.at(I) = boost::out_degree(*vi, g);
      }
    }

    return result;
  }

};
}

#endif //COMPOSITION_FRAMEWORK_COVERAGE_HPP
