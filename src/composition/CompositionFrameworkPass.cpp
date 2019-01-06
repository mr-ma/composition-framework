#include <composition/AnalysisRegistry.hpp>
#include <composition/CompositionFrameworkPass.hpp>
#include <composition/graph/constraint/true.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Performance.hpp>
#include <composition/metric/Weights.hpp>
#include <composition/strategy/Avoidance.hpp>
#include <composition/strategy/Weight.hpp>
#include <composition/support/Pass.hpp>
#include <composition/support/options.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <llvm/IR/CallSite.h>
#include <llvm/IR/Constants.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/raw_os_ostream.h>
#include <llvm/Support/raw_ostream.h>
#include <self-checksumming/FunctionFilter.h>

using namespace llvm;

namespace composition {
using graph::ManifestDependencyMap;
using graph::ManifestProtectionMap;
using graph::filter::filter_removed_graph;
using graph::util::graph_to_dot;
using graph::util::graph_to_graphml;
using support::AddCFG;
using support::cStats;
using support::DumpGraphs;
using support::DumpStats;
using support::PatchInfo;
using support::UseStrategy;
using support::WeightConfig;

static llvm::RegisterPass<CompositionFrameworkPass> X("composition-framework", "Composition Framework Pass", false,
                                                      false);

char CompositionFrameworkPass::ID = 0;

void CompositionFrameworkPass::getAnalysisUsage(llvm::AnalysisUsage& AU) const {
  AU.addRequired<BlockFrequencyInfoWrapperPass>();
  AU.addRequiredTransitive<FunctionFilterPass>();
  AU.setPreservesAll();

  auto registered = AnalysisRegistry::GetAll();
  for (auto passInfo : registered) {
    AU.addRequiredID(passInfo.ID);
  }
}

bool CompositionFrameworkPass::doInitialization(Module& M) {
  dbgs() << "AnalysisPass loaded...\n";
  // The following code loads all the tags from the source code.
  // The tags are then applied to the function for which a tag was used.
  // The tags may be used by any pass to decide if it is e.g., a sensitive
  // function Start annotations from @src:
  // http://bholt.org/posts/llvm-quick-tricks.html
  auto global_annos = M.getNamedGlobal("llvm.global.annotations");
  if (global_annos != nullptr) {
    auto a = cast<ConstantArray>(global_annos->getOperand(0));
    for (unsigned int i = 0; i < a->getNumOperands(); i++) {
      auto e = cast<ConstantStruct>(a->getOperand(i));

      if (auto fn = dyn_cast<Function>(e->getOperand(0)->getOperand(0))) {
        auto anno = cast<ConstantDataArray>(cast<GlobalVariable>(e->getOperand(1)->getOperand(0))->getOperand(0))
                        ->getAsCString();
        fn->addFnAttr(anno); // <-- add function annotation here
        dbgs() << "Sensitive function: " << fn->getName().str() << "\n";
      }
    }
  }
  // End annotations from @src: http://bholt.org/posts/llvm-quick-tricks.html
  return true;
}

std::unique_ptr<graph::ProtectionGraph> buildGraphFromManifests(std::vector<Manifest*> manifests) {
  auto g = std::make_unique<graph::ProtectionGraph>();

  cStats.proposedManifests = manifests.size();
  Profiler constructionProfiler{};
  g->addManifests(manifests);
  cStats.timeGraphConstruction += constructionProfiler.stop();
  return g;
}

void addCallGraph(std::unique_ptr<graph::ProtectionGraph>& g, llvm::Module& M) {
  if (AddCFG) {
    dbgs() << "Building CallGraph\n";
    Profiler constructionProfiler{};
    // TODO #1 It's probably better to use a callgraph here. However, using a
    // callgraph leads to a SIGSEGV for no reason Printing and dumping of the
    // callgraph works, but as soon as accessing it is tried the program behaves
    // unexpectedly. Therefore, for now, only add direct call edges as CFG part
    // to the graph.
    for (auto& F : M) {
      // Look for calls by this function.
      for (auto& BB : F) {
        for (auto& I : BB) {
          Value* v = &cast<Value>(I);
          CallSite CS(v);
          if (!CS) {
            continue;
          }

          Function* Callee = CS.getCalledFunction();
          if (!Callee) {
            continue;
          }

          // Only direct calls are possible to track
          g->addCFG(&F, Callee);
        }
      }
    }
    cStats.timeGraphConstruction += constructionProfiler.stop();
    dbgs() << "Done building CallGraph\n";
  }
}

void printGraphs(std::unique_ptr<graph::ProtectionGraph>& g, std::string name) {
  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(g->getGraph());
    graph_to_dot(g->getGraph(), name + ".dot");
    graph_to_graphml(g->getGraph(), name + ".graphml");
    graph_to_dot(fg, name + "_removed.dot");
    graph_to_graphml(fg, name + "_removed.graphml");
  }
}

void expandGraph(std::unique_ptr<graph::ProtectionGraph>& g) {
  dbgs() << "Expand graph to instructions\n";
  Profiler constructionProfiler{};
  g->expandToInstructions();
  cStats.timeGraphConstruction += constructionProfiler.stop();
  dbgs() << "Done\n";
}

void reduceGraph(std::unique_ptr<graph::ProtectionGraph>& g) {
  dbgs() << "Remove non instruction vertices from graph\n";
  Profiler constructionProfiler{};
  g->reduceToInstructions();
  cStats.timeGraphConstruction += constructionProfiler.stop();
  dbgs() << "Done\n";
}

bool CompositionFrameworkPass::runOnModule(llvm::Module& M) {
  analysisPass(M);
  graphPass(M);
  return protectionPass(M);
}

std::vector<Manifest*> CompositionFrameworkPass::SortedManifests() {
  auto manifestSet = ManifestRegistry::GetAll();

  for (auto* m : manifestSet) {
    if (m->postPatching) {
      return Graph->topologicalSortManifests(manifestSet);
    }
  }
  std::vector<Manifest*> result{manifestSet.begin(), manifestSet.end()};

  return result;
}

bool CompositionFrameworkPass::doFinalization(llvm::Module& M) {
  Graph->destroy();
  ManifestRegistry::destroy();
  return Pass::doFinalization(M);
}

bool CompositionFrameworkPass::analysisPass(llvm::Module& M) {
  dbgs() << "AnalysisPass running\n";

  auto mSet = ManifestRegistry::GetAll();
  std::vector<Manifest*> manifests{};
  manifests.insert(manifests.end(), mSet.begin(), mSet.end());

  Graph = buildGraphFromManifests(manifests);
  addCallGraph(Graph, M);
  printGraphs(Graph, "graph_raw");

  expandGraph(Graph);
  printGraphs(Graph, "graph_expanded");

  reduceGraph(Graph);
  printGraphs(Graph, "graph_reduced");

  return false;
}

bool CompositionFrameworkPass::graphPass(llvm::Module& M) {
  dbgs() << "GraphPass running\n";

  auto& filterPass = getAnalysis<FunctionFilterPass>();
  sensitiveFunctions = filterPass.get_functions_info()->get_functions();

  metric::Weights w;
  if (!WeightConfig.empty()) {
    std::ifstream ifs(WeightConfig.getValue());
    w = metric::Weights(ifs);
  }

  std::unordered_map<llvm::Function*, llvm::BlockFrequencyInfo*> BFI{};
  for (auto& F : M) {
    if (F.isDeclaration()) {
      continue;
    }
    dbgs() << F.getName() << "\n";
    auto& bfiPass = getAnalysis<BlockFrequencyInfoWrapperPass>(F);
    auto& bf = bfiPass.getBFI();
    BFI.insert({&F, &bf});
  }

  std::random_device r{};
  auto rng = std::default_random_engine{r()};

  std::unordered_map<std::string, std::unique_ptr<strategy::Strategy>> strategies;
  strategies.emplace("random", std::make_unique<strategy::Random>(rng));
  strategies.emplace("avoidance", std::make_unique<strategy::Avoidance>(strategy::Avoidance({
                                      {"cm", -1},
                                      {"oh_assert_", 1},
                                      {"oh_assert", 1},
                                      {"sroh_assert", 1},
                                      {"sroh_hash", 2},
                                      {"oh_hash_", 2},
                                      {"oh_hash", 2},
                                      {"sc", 3},
                                      {"cfi", 4},
                                  })));
  strategies.emplace("weight", std::make_unique<strategy::Weight>(w, BFI));

  if (strategies.find(UseStrategy.getValue()) == strategies.end()) {
    report_fatal_error("The given composition-framework strategy does not exist.", false);
  }

  dbgs() << "Calculating Manifest dependencies\n";
  Graph->computeManifestDependencies();
  dbgs() << "GraphPass strong_components\n";
  Graph->conflictHandling(Graph->getGraph(), strategies.at(UseStrategy.getValue()));

  printGraphs(Graph, "graph_scc");
  dbgs() << "GraphPass done\n";

  dbgs() << "Optimizing\n";
  auto manifests = SortedManifests();
  Graph->destroy();

  // Rebuild the graph so we work with a clean version
  for (auto& m : manifests) {
    for (auto& u : m->UndoValues()) {
      m->constraints.push_back(std::make_shared<graph::constraint::True>("true", u));
    }
  }
  Graph = buildGraphFromManifests(manifests);
  expandGraph(Graph);
  reduceGraph(Graph);
  Graph->computeManifestDependencies();

  std::unordered_set<llvm::Instruction*> instructions{};

  for (auto F : sensitiveFunctions) {
    auto result = composition::metric::Coverage::ValueToInstructions(F);
    instructions.insert(result.begin(), result.end());
  }

  Graph->optimizeProtections(Graph->getGraph(), BFI, &M, manifests, instructions);
  dbgs() << "Optimizing done\n";

  return false;
}

bool CompositionFrameworkPass::protectionPass(llvm::Module& M) {
  dbgs() << "ProtectionPass running\n";

  auto manifests = SortedManifests();
  assert(manifests.size() == ManifestRegistry::GetAll().size());

  dbgs() << "Got " << manifests.size() << " manifests\n";
  cStats.actualManifests = manifests.size();

  std::vector<std::pair<std::string, std::string>> patchInfos{};
  size_t i = 0;
  size_t total = manifests.size();
  for (auto* m : manifests) {
    m->Clean();
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "\r";

    m->Redo();
    if (m->postPatching) {
      patchInfos.emplace_back(m->name, m->patchInfo);
    }
  }
  dbgs() << "#" << std::to_string(i) << "/" << std::to_string(total) << "\n";

  /*
   * TODO: I discovered this possibility very late in the thesis. It can be
   * extended to have more control over pass scheduling. For example, we could
   * add a function which allows protecting arbitrary small llvm values.
   * Additionally, layering of protections can be achieved, e.g. applying SC ->
   * OH and protect both with another layer of SC explicitly.
   */
  auto registered = AnalysisRegistry::GetAll();
  for (auto a : registered) {
    auto* p = getResolver()->findImplPass(a.ID);

    if (p == nullptr) {
      llvm_unreachable("Pass no longer available...");
    }
    auto* c = static_cast<composition::support::Pass*>(p); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
    c->finalizeComposition();
  }

  dbgs() << "Writing patchinfo\n";
  writePatchInfo(patchInfos);

  dbgs() << "Collecting stats\n";
  cStats.stats.collect(sensitiveFunctions, manifests, Graph->getManifestProtectionMap());
  cStats.dump(dbgs());

  if (!DumpStats.empty()) {
    auto fdStream = std::ofstream(DumpStats.getValue(), std::ofstream::out);
    if (fdStream.good()) {
      dbgs() << "Dumping stats to file: " << DumpStats.getValue() << "\n";
      auto stream = raw_os_ostream(fdStream);
      cStats.dump(stream);
    } else {
      dbgs() << "Could not dump stats\n";
    }
    fdStream.close();
  }

  dbgs() << "Cleanup\n";
  trace::PreservedValueRegistry::Clear();
  return !manifests.empty();
}

void CompositionFrameworkPass::writePatchInfo(std::vector<std::pair<std::string, std::string>> patchInfos) {
  nlohmann::json j = patchInfos;
  std::ofstream file(PatchInfo.getValue());
  file << j.dump(4);
  file.close();
}

} // namespace composition