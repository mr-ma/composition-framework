#include <composition/support/options.hpp>

using namespace llvm;

namespace composition::support {
Stats cStats{};
cl::opt<bool> DumpGraphs("cf-dump-graphs", cl::Hidden, cl::desc("Graph files are written to disk"));
cl::opt<bool> AddCFG("cf-add-cfg", cl::Hidden, cl::desc("Adds CFG edges to graphs"));
cl::opt<std::string> WeightConfig("cf-weights", cl::Hidden,
                                  cl::desc("Weights to influence the metrics used to decide if "
                                           "a conflict is resolved."));
cl::opt<std::string> DumpStats("cf-stats", cl::Hidden,
                               cl::desc("Dumps stats about the composition to the given file."));
cl::opt<std::string> UseStrategy("cf-strategy", cl::init("random"), cl::desc("Strategy to use to resolve conflicts."));
cl::opt<std::string> PatchInfo("cf-patchinfo", cl::init("cf-patchinfo.json"),
                               cl::desc("Dumps the patching information to the given file."));

cl::opt<int> NumberOfOptimizations("cf-number-of-optimizations", cl::Hidden);
cl::opt<double> PercentageOfManifests("cf-percentage-of-manifests", cl::Hidden);
cl::opt<double> PerformanceThreshold("cf-performance-threshold", cl::Hidden);
cl::opt<double> MinConnectivity("cf-min-connectivity", cl::Hidden);
cl::opt<double> MinCoverage("cf-min-coverage", cl::Hidden);
cl::opt<double> MinImplicitCoverage("cf-min-implicit-coverage", cl::Hidden);
} // namespace composition::support