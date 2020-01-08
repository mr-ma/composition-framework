#include <composition/support/options.hpp>

namespace composition::support {
Stats cStats{};
llvm::cl::opt<std::string> DumpGraphs("cf-dump-graphs", llvm::cl::Hidden, llvm::cl::desc("Graph files are written to disk"));
llvm::cl::opt<bool> AddCFG("cf-add-cfg", llvm::cl::Hidden, llvm::cl::desc("Adds CFG edges to graphs"));
llvm::cl::opt<std::string> WeightConfig("cf-weights", llvm::cl::Hidden,
                                        llvm::cl::desc("Weights to influence the metrics used to decide if "
                                                       "a conflict is resolved."));
llvm::cl::opt<std::string> DumpStats("cf-stats", llvm::cl::Hidden,
                                     llvm::cl::desc("Dumps stats about the composition to the given file."));
llvm::cl::opt<std::string>
    UseStrategy("cf-strategy", llvm::cl::init("random"), llvm::cl::desc("Strategy to use to resolve conflicts."));
llvm::cl::opt<std::string> PatchInfo("cf-patchinfo", llvm::cl::init("cf-patchinfo.json"),
                                     llvm::cl::desc("Dumps the patching information to the given file."));

llvm::cl::opt<std::string> ILPProblem("cf-ilp-prob", llvm::cl::Hidden);
llvm::cl::opt<std::string> ILPSolution("cf-ilp-sol", llvm::cl::Hidden);
llvm::cl::opt<std::string> ILPSolutionReadable("cf-ilp-sol-readable", llvm::cl::Hidden);
llvm::cl::opt<std::string> ILPObjective("cf-ilp-obj", llvm::cl::init("overhead"), llvm::cl::desc("ILP objective function choose between min 'overhead' (default),  max 'explicit', max 'implicit', max 'connectivity'"));

/*
 * List of ILP options
 */

//llvm::cl::opt<int>
  //  ILPConnectivityBound("cf-ilp-connectivity", llvm::cl::init(2), llvm::cl::desc("Instruction Connectivity"));
llvm::cl::opt<double>
    ILPBlockConnectivityBound("cf-ilp-blockconnectivity-bound", llvm::cl::init(0), llvm::cl::desc("Block Connectivity"));
llvm::cl::opt<int>
    ILPImplicitBound("cf-ilp-implicit-bound", llvm::cl::init(0), llvm::cl::desc("Implicit Coverage"));
llvm::cl::opt<double>
    ILPConnectivityBound("cf-ilp-connectivity-bound", llvm::cl::init(0), llvm::cl::desc("Instruction Connectivity"));
llvm::cl::opt<int>
    ILPExplicitBound("cf-ilp-explicit-bound", llvm::cl::init(0), llvm::cl::desc("Explicit coverage constraint"));
llvm::cl::opt<double>
    ILPOverheadBound("cf-ilp-overhead-bound", llvm::cl::init(0), llvm::cl::desc("Overhead constraint"));


llvm::cl::opt<bool> ExperimentalSCIPSolver("cf-experimental-scip", llvm::cl::Hidden, llvm::cl::desc("Use SCIP solver to solve the problem in a hacky way!"));


llvm::cl::opt<bool> ExperimentalNetworkX("cf-experimental-networkx", llvm::cl::Hidden, llvm::cl::desc("Use python NetworkX to find cycles in the protection graph."));

llvm::cl::opt<std::string> ExperimentalNetworkXEdgeFile("cf-experimental-networkx-edgefile", llvm::cl::Hidden);
llvm::cl::opt<std::string> ExperimentalNetworkXCycleFile("cf-experimental-networkx-cyclefile", llvm::cl::Hidden);


} // namespace composition::support
