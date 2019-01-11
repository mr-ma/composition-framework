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
llvm::cl::opt<std::string> UseStrategy("cf-strategy", llvm::cl::init("random"), llvm::cl::desc("Strategy to use to resolve conflicts."));
llvm::cl::opt<std::string> PatchInfo("cf-patchinfo", llvm::cl::init("cf-patchinfo.json"),
                               llvm::cl::desc("Dumps the patching information to the given file."));

llvm::cl::opt<int> NumberOfOptimizations("cf-number-of-optimizations", llvm::cl::Hidden);
llvm::cl::opt<double> PercentageOfManifests("cf-percentage-of-manifests", llvm::cl::Hidden);
llvm::cl::opt<double> PerformanceThreshold("cf-performance-threshold", llvm::cl::Hidden);
llvm::cl::opt<double> MinConnectivity("cf-min-connectivity", llvm::cl::Hidden);
llvm::cl::opt<double> MinCoverage("cf-min-coverage", llvm::cl::Hidden);
llvm::cl::opt<double> MinImplicitCoverage("cf-min-implicit-coverage", llvm::cl::Hidden);
} // namespace composition::support