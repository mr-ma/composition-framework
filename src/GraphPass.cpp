#include <llvm/Support/Debug.h>

#include <composition/GraphPass.hpp>
#include <composition/AnalysisPass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/graph/dot.hpp>
#include <composition/graph/graphml.hpp>

using namespace llvm;
namespace composition {

void GraphPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequired<AnalysisPass>();
}

bool GraphPass::runOnModule(llvm::Module &module) {
  dbgs() << "GraphPass running\n";

  auto &pass = getAnalysis<AnalysisPass>();
  Graph = std::move(pass.getGraph());
  dbgs() << "GraphPass SCC\n";
  dbgs() << "Got " << (*ManifestRegistry::GetAll()).size() << " manifests\n";

  Graph.SCC_DEPENDENCY(Graph.getGraph());
  dbgs() << "Got " << (*ManifestRegistry::GetAll()).size() << " manifests\n";
  save_graph_to_dot(Graph.getGraph(), "graph_scc.dot");
  save_graph_to_graphml(Graph.getGraph(), "graph_scc.graphml");
  // Get all registered analysis passes and check if one needs postpatching
  // If a pass needs postpatching then apply topological sorting before applying the protections
  auto registered = AnalysisRegistry::GetAll();
  for (const auto &passInfo : registered) {
    if (passInfo.second) {
      dbgs() << "SORTING:\n";
      //TODO fix topo sort for filtered multi edge graph
      /*auto sorted = Graph.topologicalSortProtections();
      for(auto vd : sorted) {
          dbgs() << g[vd].name << "\n";
      }*/
      break;
    }
  }
  dbgs() << "GraphPass done\n";

  //TODO create postpatching manifest order/export to json
  return false;
}

char GraphPass::ID = 0;

std::unordered_map<ManifestIndex, Manifest> GraphPass::GetManifestsInOrder() {
  return std::unordered_map<ManifestIndex, Manifest>(*ManifestRegistry::GetAll());
}

static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);
}