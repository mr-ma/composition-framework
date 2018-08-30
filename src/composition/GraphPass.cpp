#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
//#include <llvm/Analysis/LazyBlockFrequencyInfo.h>
#include <llvm/PassAnalysisSupport.h>
#include <composition/options.hpp>
#include <composition/GraphPass.hpp>
#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Weights.hpp>

using namespace llvm;

namespace composition {

static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass", true, true);

char GraphPass::ID = 0;

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.addRequiredTransitive<AnalysisPass>();
  //AU.addRequired<LazyBlockFrequencyInfoPass>();
  AU.setPreservesAll();
}

bool GraphPass::runOnModule(llvm::Module &M) {
  dbgs() << "GraphPass running\n";

  Weights w;
  if(!WeightConfig.empty()) {
    std::ifstream ifs(WeightConfig.getValue());
    w = Weights(ifs);
  }
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

  Graph->conflictHandling(Graph->getGraph());

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    save_graph_to_dot(Graph->getGraph(), "graph_scc.dot");
    save_graph_to_graphml(Graph->getGraph(), "graph_scc.graphml");
    save_graph_to_dot(fg, "graph_scc_removed.dot");
    save_graph_to_graphml(fg, "graph_scc_removed.graphml");
  }
  dbgs() << "GraphPass done\n";

  //TODO Optimize, i.e., remove manifests to make the application run faster
  //TODO Block places to prevent from obfuscation

  return false;
}

std::vector<std::shared_ptr<Manifest>> GraphPass::SortedManifests() {
  auto manifestSet = ManifestRegistry::GetAll();
  std::vector<std::shared_ptr<Manifest>> manifests{manifestSet.begin(), manifestSet.end()};

  for (const auto &m : manifestSet) {
    if (m->postPatching) {
      return Graph->topologicalSortManifests(manifests);
    }
  }

  return manifests;
}

bool GraphPass::doFinalization(Module &module) {
  Graph->destroy();
  ManifestRegistry::destroy();
  return Pass::doFinalization(module);
}
}