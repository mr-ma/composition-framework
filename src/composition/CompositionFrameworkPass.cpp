#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/raw_os_ostream.h>
#include <llvm/Support/Error.h>
#include <llvm/IR/CallSite.h>
#include <llvm/IR/Constants.h>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <self-checksumming/FunctionFilter.h>
#include <composition/support/options.hpp>
#include <composition/CompositionFrameworkPass.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Weights.hpp>
#include <composition/strategy/Avoidance.hpp>
#include <composition/strategy/Weight.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/metric/Performance.hpp>
#include <composition/support/Pass.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>

using namespace llvm;

namespace composition {
using graph::ManifestDependencyMap;
using graph::ManifestProtectionMap;
using graph::filter::filter_removed_graph;
using graph::util::graph_to_graphml;
using graph::util::graph_to_dot;
using support::WeightConfig;
using support::UseStrategy;
using support::DumpGraphs;
using support::PatchInfo;
using support::cStats;
using support::AddCFG;
using support::DumpStats;

static llvm::RegisterPass<CompositionFrameworkPass> X(
    "composition-framework",
    "Composition Framework Pass",
    false,
    false
);

char CompositionFrameworkPass::ID = 0;

void CompositionFrameworkPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.addRequired<BlockFrequencyInfoWrapperPass>();
  AU.addRequiredTransitive<FunctionFilterPass>();
  AU.setPreservesAll();

  auto registered = AnalysisRegistry::GetAll();
  for (auto passInfo : registered) {
    AU.addRequiredID(passInfo.ID);
  }
}

bool CompositionFrameworkPass::doInitialization(Module &M) {
  dbgs() << "AnalysisPass loaded...\n";
  Graph = std::make_unique<graph::ProtectionGraph>();

  //The following code loads all the tags from the source code.
  //The tags are then applied to the function for which a tag was used.
  //The tags may be used by any pass to decide if it is e.g., a sensitive function
  //Start annotations from @src: http://bholt.org/posts/llvm-quick-tricks.html
  auto global_annos = M.getNamedGlobal("llvm.global.annotations");
  if (global_annos) {
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
  //End annotations from @src: http://bholt.org/posts/llvm-quick-tricks.html
  return true;
}

bool CompositionFrameworkPass::runOnModule(llvm::Module &M) {
  analysisPass(M);
  graphPass(M);

  return protectionPass(M);
}

std::vector<Manifest *> CompositionFrameworkPass::SortedManifests() {
  auto manifestSet = ManifestRegistry::GetAll();

  for (auto *m : manifestSet) {
    if (m->postPatching) {
      return Graph->topologicalSortManifests(manifestSet);
    }
  }
  std::vector<Manifest *> result{manifestSet.begin(), manifestSet.end()};

  return result;
}

bool CompositionFrameworkPass::doFinalization(Module &module) {
  Graph->destroy();
  ManifestRegistry::destroy();
  return Pass::doFinalization(module);
}

bool CompositionFrameworkPass::analysisPass(llvm::Module &M) {
  dbgs() << "AnalysisPass running\n";

  auto manifests = ManifestRegistry::GetAll();
  size_t total = manifests.size();
  dbgs() << "Adding " << std::to_string(total) << " manifests to protection graph\n";
  cStats.proposedManifests = total;
  size_t i = 0;
  Profiler constructionProfiler{};
  for (auto &m : manifests) {
    if (!m->Clean()) {
      ManifestRegistry::Remove(m);
      continue;
    }
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "\r";
    for (auto it = m->constraints.begin(), it_end = m->constraints.end(); it != it_end; ++it) {
      Graph->addConstraint(m, (*it));
    }
  }
  cStats.timeGraphConstruction += constructionProfiler.stop();
  dbgs() << "#" << std::to_string(i) << "/" << std::to_string(total) << "\n";

  if (AddCFG) {
    dbgs() << "Building CallGraph\n";
    constructionProfiler.reset();
    //TODO #1 It's probably better to use a callgraph here. However, using a callgraph leads to a SIGSEGV for no reason
    //Printing and dumping of the callgraph works, but as soon as accessing it is tried the program behaves unexpectedly.
    //Therefore, for now, only add direct call edges as CFG part to the graph.
    for (auto &F : M) {
      // Look for calls by this function.
      for (auto &BB : F) {
        for (auto &I : BB) {
          Value *v = &cast<Value>(I);
          CallSite CS(v);
          if (!CS) {
            continue;
          }

          Function *Callee = CS.getCalledFunction();
          if (!Callee) {
            continue;
          }

          //Only direct calls are possible to track
          Graph->addCFG(&F, Callee);
        }
      }
    }
    cStats.timeGraphConstruction += constructionProfiler.stop();
    dbgs() << "Done building CallGraph\n";
  }

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    graph_to_dot(Graph->getGraph(), "graph_raw.dot");
    graph_to_graphml(Graph->getGraph(), "graph_raw.graphml");
    graph_to_dot(fg, "graph_raw_removed.dot");
    graph_to_graphml(fg, "graph_raw_removed.graphml");
  }

  dbgs() << "Expand graph to instructions\n";
  constructionProfiler.reset();
  Graph->expandToInstructions();
  cStats.timeGraphConstruction += constructionProfiler.stop();
  dbgs() << "Done\n";

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    graph_to_dot(Graph->getGraph(), "graph_expanded.dot");
    graph_to_graphml(Graph->getGraph(), "graph_expanded.graphml");
    graph_to_dot(fg, "graph_expanded_removed.dot");
    graph_to_graphml(fg, "graph_expanded_removed.graphml");
  }

  dbgs() << "Remove non instruction vertices from graph\n";
  constructionProfiler.reset();
  Graph->reduceToInstructions();
  cStats.timeGraphConstruction += constructionProfiler.stop();
  dbgs() << "Done\n";

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    graph_to_dot(Graph->getGraph(), "graph_reduced.dot");
    graph_to_graphml(Graph->getGraph(), "graph_reduced.graphml");
    graph_to_dot(fg, "graph_reduced_removed.dot");
    graph_to_graphml(fg, "graph_reduced_removed.graphml");
  }
  return false;
}

bool CompositionFrameworkPass::graphPass(llvm::Module &M) {
  dbgs() << "GraphPass running\n";

  auto &filterPass = getAnalysis<FunctionFilterPass>();
  sensitiveFunctions = filterPass.get_functions_info()->get_functions();

  metric::Weights w;
  if (!WeightConfig.empty()) {
    std::ifstream ifs(WeightConfig.getValue());
    w = metric::Weights(ifs);
  }

  std::unordered_map<llvm::Function *, llvm::BlockFrequencyInfo *> BFI{};
  for (auto &F : M) {
    if (F.isDeclaration()) {
      continue;
    }
    dbgs() << F.getName() << "\n";
    auto &bfiPass = getAnalysis<BlockFrequencyInfoWrapperPass>(F);
    auto &bf = bfiPass.getBFI();
    BFI.insert({&F, &bf});
  }

  std::random_device r{};
  auto rng = std::default_random_engine{r()};

  std::unordered_map<std::string, std::unique_ptr<strategy::Strategy>> strategies;
  strategies.emplace("random", std::make_unique<strategy::Random>(rng));
  strategies.emplace("avoidance", std::make_unique<strategy::Avoidance>(strategy::Avoidance(
      {
          {"cm", -1},
          {"oh_assert_", 1},
          {"oh_assert", 1},
          {"sroh_assert", 1},
          {"sroh_hash", 2},
          {"oh_hash_", 2},
          {"oh_hash", 2},
          {"sc", 3},
          {"cfi", 4},
      }
  )));
  strategies.emplace("weight", std::make_unique<strategy::Weight>(
      w,
      BFI
  ));

  if (strategies.find(UseStrategy.getValue()) == strategies.end()) {
    report_fatal_error("The given composition-framework strategy does not exist.", false);
  }

  dbgs() << "Calculating Manifest dependencies\n";
  Graph->computeManifestDependencies();
  dbgs() << "GraphPass strong_components\n";
  Graph->conflictHandling(Graph->getGraph(), strategies.at(UseStrategy.getValue()));

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    graph_to_dot(Graph->getGraph(), "graph_scc.dot");
    graph_to_graphml(Graph->getGraph(), "graph_scc.graphml");
    graph_to_dot(fg, "graph_scc_removed.dot");
    graph_to_graphml(fg, "graph_scc_removed.graphml");
  }
  dbgs() << "GraphPass done\n";

  return false;
}

bool CompositionFrameworkPass::protectionPass(llvm::Module &M) {
  dbgs() << "ProtectionPass running\n";

  auto manifests = SortedManifests();
  assert(manifests.size() == ManifestRegistry::GetAll().size());

  dbgs() << "Got " << manifests.size() << " manifests\n";
  cStats.actualManifests = manifests.size();

  std::vector<std::pair<std::string, std::string>> patchInfos{};
  size_t i = 0;
  size_t total = manifests.size();
  for (auto *m : manifests) {
    if (!m->Clean()) {
      llvm_unreachable("Manifest not clean...");
    }
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "\r";

    m->Redo();
    if (m->postPatching) {
      patchInfos.emplace_back(m->name, m->patchInfo);
    }
  }
  dbgs() << "#" << std::to_string(i) << "/" << std::to_string(total) << "\n";

  /*
   * TODO: I discovered this possibility very late in the thesis. It can be extended to have more control over pass scheduling.
   * For example, we could add a function which allows protecting arbitrary small llvm values.
   * Additionally, layering of protections can be achieved, e.g. applying SC -> OH and protect both with another layer of SC explicitly.
   */
  auto registered = AnalysisRegistry::GetAll();
  for (auto a : registered) {
    auto *p = getResolver()->findImplPass(a.ID);

    if (p == nullptr) {
      llvm_unreachable("Pass no longer available...");
    }
    auto *c = static_cast<composition::support::Pass *>(p);
    c->finalizeComposition();
  }

  writePatchInfo(patchInfos);

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

  trace::PreservedValueRegistry::Clear();
  return !manifests.empty();
}

void CompositionFrameworkPass::writePatchInfo(std::vector<std::pair<std::string, std::string>> patchInfos) {
  nlohmann::json j = patchInfos;
  std::ofstream file(PatchInfo.getValue());
  file << j.dump(4);
  file.close();
}

}