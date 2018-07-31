#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
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

  Graph.SCC_DEPENDENCY(Graph.getGraph());

  auto fg = filter_removed_graph(Graph.getGraph());
  save_graph_to_dot(Graph.getGraph(), "graph_scc.dot");
  save_graph_to_graphml(Graph.getGraph(), "graph_scc.graphml");
  save_graph_to_dot(fg, "graph_scc_removed.dot");
  save_graph_to_graphml(fg, "graph_scc_removed.graphml");
  dbgs() << "GraphPass done\n";

  //TODO create postpatching manifest order/export to json
  return false;
}

char GraphPass::ID = 0;

std::vector<Manifest> GraphPass::GetManifestsInOrder() {
  bool requireTopologicalSort = false;

  auto m = *ManifestRegistry::GetAll();
  for (const auto &kv : m) {
    if (kv.second.postPatching) {
      requireTopologicalSort = true;
      break;
    }
  }

  auto indexes = Graph.manifestIndexes(requireTopologicalSort);
  auto result = std::vector<Manifest>();
  std::transform(std::begin(indexes),
                 std::end(indexes),
                 std::back_inserter(result),
                 [m](const auto i) {
                   return m.find(i)->second;
                 });

  return result;
}

static llvm::RegisterPass<GraphPass> X("constraint-graph", "Constraint Graph Pass",
                                       false /* Only looks at CFG */,
                                       true /* Analysis Pass */);
}