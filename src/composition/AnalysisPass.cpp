#include <llvm/IR/CallSite.h>
#include <llvm/Analysis/CallGraph.h>
#include <composition/options.hpp>
#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/trace/TraceableValue.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>

using namespace llvm;
namespace composition {
char AnalysisPass::ID = 0;

bool AnalysisPass::doInitialization(Module &M) {
  dbgs() << "AnalysisPass loaded...\n";
  Graph = std::make_unique<ProtectionGraph>();

  //The following code loads all the tags from the source code.
  //The tags are then applied to the function for which a tag was used.
  //The tags may be used by any pass to decide if it is e.g., a sensitive function
  //Start annotations from @ http://bholt.org/posts/llvm-quick-tricks.html
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
  //End annotations from @ http://bholt.org/posts/llvm-quick-tricks.html
  return true;
}

void AnalysisPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
  dbgs() << "Called analysis usage\n";
  auto registered = AnalysisRegistry::GetAll();
  dbgs() << "Got " << std::to_string(registered.size()) << "\n";

  for (auto passInfo : registered) {
    dbgs() << "Require " << std::to_string(reinterpret_cast<uintptr_t>(passInfo.ID)) << "\n";
    AU.addRequiredID(passInfo.ID);
  }
}

bool AnalysisPass::runOnModule(llvm::Module &M) {
  dbgs() << "AnalysisPass running\n";

  auto manifests = ManifestRegistry::GetAll();
  size_t total = manifests.size();
  dbgs() << "Adding " << std::to_string(total) << " manifests to protection graph\n";
  size_t i = 0;
  for (auto &m : manifests) {
    dbgs() << "#" << std::to_string(i++) << "/" << std::to_string(total) << "\r";
    for (const auto &c : m->constraints) {
      Graph->addConstraint(m, c);
    }
    m->dump();
  }
  dbgs() << "#" << std::to_string(i) << "/" << std::to_string(total) << "\n";

  if(AddCFG) {
    dbgs() << "Building CallGraph\n";

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
    dbgs() << "Done building CallGraph\n";
  }

  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    save_graph_to_dot(Graph->getGraph(), "graph_raw.dot");
    save_graph_to_graphml(Graph->getGraph(), "graph_raw.graphml");
    save_graph_to_dot(fg, "graph_raw_removed.dot");
    save_graph_to_graphml(fg, "graph_raw_removed.graphml");
  }
  dbgs() << "Expand graph to instructions\n";
  //Graph.expandToFunctions();
  Graph->expandToInstructions();
  dbgs() << "Done\n";
  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    save_graph_to_dot(Graph->getGraph(), "graph_expanded.dot");
    save_graph_to_graphml(Graph->getGraph(), "graph_expanded.graphml");
    save_graph_to_dot(fg, "graph_expanded_removed.dot");
    save_graph_to_graphml(fg, "graph_expanded_removed.graphml");
  }
  dbgs() << "Remove non instruction vertices from graph\n";
  //Graph.reduceToFunctions();
  Graph->reduceToInstructions();
  dbgs() << "Done\n";
  if (DumpGraphs) {
    dbgs() << "Writing graphs\n";
    auto fg = filter_removed_graph(Graph->getGraph());
    save_graph_to_dot(Graph->getGraph(), "graph_reduced.dot");
    save_graph_to_graphml(Graph->getGraph(), "graph_reduced.graphml");
    save_graph_to_dot(fg, "graph_reduced_removed.dot");
    save_graph_to_graphml(fg, "graph_reduced_removed.graphml");
  }
  return false;
}

std::unique_ptr<ProtectionGraph> AnalysisPass::getGraph() {
  return std::move(Graph);
}

static RegisterPass<AnalysisPass> X("constraint-analysis", "Constraint Analysis Pass", true, true);
}