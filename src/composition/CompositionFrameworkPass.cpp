#include <composition/AnalysisRegistry.hpp>
#include <composition/CompositionFrameworkPass.hpp>
#include <composition/graph/constraint/dependency.hpp>
#include <composition/graph/constraint/true.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Performance.hpp>
#include <composition/metric/Weights.hpp>
#include <composition/metric/Coverage.hpp>
#include <composition/support/Pass.hpp>
#include <composition/support/options.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>
#include <function-filter/Filter.hpp>
#include <llvm/Analysis/BlockFrequencyInfo.h>
#include <llvm/Analysis/BranchProbabilityInfo.h>
#include <llvm/IR/CallSite.h>
#include <llvm/IR/Constants.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/Error.h>
//#include <llvm/Support/raw_os_ostream.h>
#include <llvm/Support/raw_ostream.h>

namespace composition {
using composition::graph::ManifestDependencyMap;
using composition::graph::ManifestProtectionMap;
using composition::graph::util::graph_to_dot;
using composition::graph::util::graph_to_graphml;
using composition::metric::Performance;
using composition::metric::Coverage;
using composition::support::AddCFG;
using composition::support::cStats;
using composition::support::DumpGraphs;
using composition::support::DumpStats;
using composition::support::PatchInfo;
using composition::support::UseStrategy;
using composition::support::WeightConfig;
using llvm::BasicBlock;
using llvm::BlockFrequencyInfoWrapperPass;
using llvm::BranchProbabilityInfoWrapperPass;
using llvm::CallSite;
using llvm::cast;
using llvm::ConstantArray;
using llvm::ConstantDataArray;
using llvm::ConstantInt;
using llvm::ConstantStruct;
using llvm::dbgs;
using llvm::dyn_cast;
using llvm::Function;
using llvm::GlobalVariable;
using llvm::LLVMContext;
using llvm::Module;
//using llvm::raw_os_ostream;
using llvm::report_fatal_error;
using llvm::ReturnInst;
using llvm::Type;
using llvm::Value;

static llvm::RegisterPass<CompositionFrameworkPass> X("composition-framework", "Composition Framework Pass", false,
                                                      false);

char CompositionFrameworkPass::ID = 0;

void CompositionFrameworkPass::getAnalysisUsage(llvm::AnalysisUsage& AU) const {
  AU.addRequired<BlockFrequencyInfoWrapperPass>();
  AU.addRequired<BranchProbabilityInfoWrapperPass>();
  AU.addRequiredTransitive<FunctionFilterPass>();
  AU.setPreservesAll();

  auto registered = AnalysisRegistry::GetAll();
  for (auto passInfo : registered) {
    AU.addRequiredID(passInfo.ID);
  }
}

void test() {
  composition::graph::ProtectionGraph G{};
  LLVMContext Context;
  std::unique_ptr<llvm::Module> M(new Module("test", Context));

  Type* ty = Type::getInt64Ty(Context);
  llvm::Function* f1 = cast<Function>(M->getOrInsertFunction("f1", ty, ty, nullptr));
  llvm::Function* f2 = cast<Function>(M->getOrInsertFunction("f2", ty, ty, nullptr));
  BasicBlock* bb2 = BasicBlock::Create(Context, "EntryBlock", f2);
  Value* One = ConstantInt::get(ty, 1);
  Value* ret = ReturnInst::Create(Context, One, bb2);

  auto m1 = new Manifest("sc-1", f2, {}, {},
                         {std::make_shared<composition::graph::constraint::Dependency>("sc-1", f1, f2)}, true, {});
  auto m2 = new Manifest("sc-2", f1, {}, {},
                         {std::make_shared<composition::graph::constraint::Dependency>("sc-2", f2, f1)}, true, {});
  std::set<Manifest*> manifests = {m1, m2};
  G.addManifests(manifests);
  G.Print("nohierarchy");
  G.addHierarchy(*M);
  G.Print("noshadow");
  G.connectShadowNodes();
  G.Print("full");
  G.topologicalSortManifests(manifests);
  dbgs() << "Working?\n";
}

bool CompositionFrameworkPass::doInitialization(Module& M) {
  dbgs() << "AnalysisPass loaded...\n";
  // test();
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

std::unique_ptr<graph::ProtectionGraph> buildGraphFromManifests(const std::set<Manifest*>& manifests) {
  auto g = std::make_unique<graph::ProtectionGraph>();

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
          // g->addCFG(&F, Callee);
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
    g->Print(name);
  }
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
  cStats.proposedManifests = mSet.size();
  Graph = buildGraphFromManifests(mSet);
  addCallGraph(Graph, M);
  Graph->addHierarchy(M);
  Graph->connectShadowNodes();
  cStats.vertices = Graph->countVertices();
  cStats.edges = Graph->countEdges();
  printGraphs(Graph, "graph_raw");

  return false;
}

bool CompositionFrameworkPass::graphPass(llvm::Module& M) {
  dbgs() << "GraphPass running\n";

  auto& filterPass = getAnalysis<FunctionFilterPass>();
  auto sFunc = filterPass.get_functions_info()->get_functions();
  sensitiveFunctions = {sFunc.begin(), sFunc.end()};

  metric::Weights w;
  if (!WeightConfig.empty()) {
    std::ifstream ifs(WeightConfig.getValue());
    w = metric::Weights(ifs);
  }

  std::unordered_map<llvm::BasicBlock*, uint64_t> BFI{};
  for (auto& F : M) {
    if (F.isDeclaration()) {
      continue;
    }

    auto& bfiPass = getAnalysis<BlockFrequencyInfoWrapperPass>(F);
    auto& bf = bfiPass.getBFI();
    for (auto& BB : F) {
      BFI.insert({&BB, Performance::getBlockFreq(&BB, &bf, false)});
    }
  }

  dbgs() << "Calculating Manifest dependencies\n";
  Graph->computeManifestDependencies();
  size_t totalInstructions = 0;
  for (auto &F: sensitiveFunctions)
    totalInstructions += Coverage::ValueToInstructions(F).size();
  dbgs() << "Running ILP\n on "<<totalInstructions<<"\n";
  auto accepted = Graph->ilpConflictHandling(M, BFI, totalInstructions);
  //auto accepted = Graph->randomConflictHandling(M);

  dbgs() << "Removing unselected manifests\n";
  // Just keep accepted manifests
  std::set<Manifest*> registered = ManifestRegistry::GetAll();
  for (auto& m : registered) {
    if (accepted.find(m) == accepted.end()) {
      ManifestRegistry::Remove(m);
    }
  }

  Graph->destroy();
  Graph = buildGraphFromManifests(accepted);
  Graph->addHierarchy(M);
  Graph->connectShadowNodes();
  Graph->computeManifestDependencies();
  cStats.stats.setManifests(accepted);

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
  //cStats.dump(dbgs());

  if (!DumpStats.empty()) {
    auto fdStream = std::ofstream(DumpStats.getValue(), std::ofstream::out);
    if (fdStream.good()) {
      dbgs() << "Dumping stats to file: " << DumpStats.getValue() << "\n";
      //auto stream = raw_os_ostream(fdStream);
      cStats.dump(fdStream);
    } else {
      dbgs() << "Could not dump stats\n";
    }
    fdStream.close();
  }
  Graph->destroy();

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
