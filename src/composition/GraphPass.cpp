#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
//#include <llvm/Analysis/LazyBlockFrequencyInfo.h>
#include <llvm/PassAnalysisSupport.h>
#include <llvm/PassSupport.h>
#include <composition/options.hpp>
#include <composition/GraphPass.hpp>
#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>

using namespace llvm;

namespace composition {

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  dbgs() << "Ana usage Graph called\n";
  ModulePass::getAnalysisUsage(AU);
  AU.addRequiredTransitive<AnalysisPass>();
  //AU.addRequired<LazyBlockFrequencyInfoPass>();
  AU.setPreservesAll();
}

bool GraphPass::runOnModule(llvm::Module &M) {
  dbgs() << "GraphPass running\n";

  /*for(auto &F : M) {
    if(F.isDeclaration()) {
      continue;
    }
    dbgs() << F.getName() << "\n";
    auto &bfiPass = getAnalysis<LazyBlockFrequencyInfoPass>(F);
    auto &lazy = bfiPass.getBFI();
    lazy.print(dbgs());
  }*/

  auto &pass = getAnalysis<AnalysisPass>();
  Graph = std::move(pass.getGraph());
  dbgs() << "GraphPass strong_components\n";

  Graph->dependencyConflictHandling(Graph->getGraph());

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    save_graph_to_dot(Graph->getGraph(), "graph_scc.dot");
    save_graph_to_graphml(Graph->getGraph(), "graph_scc.graphml");
    save_graph_to_dot(fg, "graph_scc_removed.dot");
    save_graph_to_graphml(fg, "graph_scc_removed.graphml");
  }
  dbgs() << "GraphPass done\n";

  //TODO create postpatching manifest order/export to json
  return false;
}

char GraphPass::ID = 0;

std::vector<std::shared_ptr<Manifest>> GraphPass::GetManifestsInOrder() {
  bool requireTopologicalSort = false;

  auto m = ManifestRegistry::GetAll();
  const auto input_size = m.size();
  for (const auto &kv : m) {
    if (kv.second->postPatching) {
      requireTopologicalSort = true;
      break;
    }
  }

  auto indexes = Graph->manifestIndexes(requireTopologicalSort);
  auto result = std::vector<std::shared_ptr<Manifest>>();
  std::transform(std::begin(indexes),
                 std::end(indexes),
                 std::back_inserter(result),
                 [m](const auto i) {
                   return m.find(i)->second;
                 });

  assert(input_size == result.size());
  return result;
}

bool GraphPass::doFinalization(Module &module) {
  Graph->destroy();
  ManifestRegistry::destroy();
  return Pass::doFinalization(module);
}

static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);
}