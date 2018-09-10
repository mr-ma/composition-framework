#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Error.h>
#include <llvm/Analysis/LazyBlockFrequencyInfo.h>
#include <composition/options.hpp>
#include <composition/GraphPass.hpp>
#include <composition/AnalysisPass.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Weights.hpp>
#include <composition/strategy/Avoidance.hpp>
#include <composition/strategy/Weight.hpp>

using namespace llvm;

namespace composition {

static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass", true, true);

char GraphPass::ID = 0;

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.addRequiredTransitive<AnalysisPass>();
  AU.addRequired<LazyBlockFrequencyInfoPass>();
  AU.setPreservesAll();
}

bool GraphPass::runOnModule(llvm::Module &M) {
  dbgs() << "GraphPass running\n";

  //cantFail(M.materializeAll(), "Materialize failed\n");

  Weights w;
  if (!WeightConfig.empty()) {
    std::ifstream ifs(WeightConfig.getValue());
    w = Weights(ifs);
  }

  std::unordered_map<llvm::Function *, llvm::BlockFrequencyInfo *> BFI{};
  for (auto &F : M) {
    if (F.isDeclaration()) {
      continue;
    }
    dbgs() << F.getName() << "\n";
    auto &bfiPass = getAnalysis<LazyBlockFrequencyInfoPass>(F);
    auto &lazy = bfiPass.getBFI();
    lazy.print(dbgs());
    BFI.insert({&F, &lazy});
  }

  std::random_device r{};
  auto rng = std::default_random_engine{r()};

  std::unordered_map<std::string, std::unique_ptr<Strategy>> strategies;
  strategies.emplace("random", std::make_unique<Random>(rng));
  strategies.emplace("avoidance", std::make_unique<Avoidance>(Avoidance(
      {
          {"cm", -1},
          {"sc", 0},
          {"cfi", 0},
          {"oh_hash_", 1},
          {"oh_hash", 1},
          {"oh_assert_", 2},
          {"oh_assert", 2},
          {"sroh_hash", 1},
          {"sroh_assert", 2},
      }
  )));
  strategies.emplace("weight", std::make_unique<Weight>(
      w,
      BFI
  ));

  if (strategies.find(UseStrategy.getValue()) == strategies.end()) {
    report_fatal_error("The given composition-framework strategy does not exist.", false);
  }

  auto &pass = getAnalysis<AnalysisPass>();
  Graph = pass.getGraph();
  dbgs() << "Calculating Manifest dependencies\n";
  Graph->computeManifestDependencies();
  dbgs() << "GraphPass strong_components\n";
  Graph->conflictHandling(Graph->getGraph(), strategies.at(UseStrategy.getValue()));

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