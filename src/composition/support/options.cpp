#include <composition/support/options.hpp>

namespace composition::support {
Stats cStats{};
llvm::cl::opt<bool> DumpGraphs("cf-dump-graphs", llvm::cl::Hidden, llvm::cl::desc("Graph files are written to disk"));
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

/*
 * List of ILP options
 */

llvm::cl::opt<int>
    DesiredConnectivity("cf-ilp-connectivity", llvm::cl::init(2), llvm::cl::desc("Instruction Connectivity"));
llvm::cl::opt<int>
    DesiredBlockConnectivity("cf-ilp-blockconnectivity", llvm::cl::init(1), llvm::cl::desc("Block Connectivity"));
llvm::cl::opt<int>
    DesiredImplicitCoverage("cf-ilp-implicitcoverage", llvm::cl::init(1), llvm::cl::desc("Implicit Coverage"));
} // namespace composition::support