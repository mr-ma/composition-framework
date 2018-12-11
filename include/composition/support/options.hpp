#ifndef COMPOSITION_FRAMEWORK_SUPPORT_OPTIONS_HPP
#define COMPOSITION_FRAMEWORK_SUPPORT_OPTIONS_HPP

#include <composition/Stats.hpp>
#include <llvm/Support/CommandLine.h>

namespace composition::support {
/*
 * List of global variables which can be used from anywhere in the framework
 */
extern Stats cStats;
extern llvm::cl::opt<bool> DumpGraphs;
extern llvm::cl::opt<bool> AddCFG;
extern llvm::cl::opt<std::string> WeightConfig;
extern llvm::cl::opt<std::string> DumpStats;
extern llvm::cl::opt<std::string> UseStrategy;
extern llvm::cl::opt<std::string> PatchInfo;
extern llvm::cl::opt<int> NumberOfOptimizations;
extern llvm::cl::opt<double> PercentageOfManifests;
extern llvm::cl::opt<double> PerformanceThreshold;
extern llvm::cl::opt<double> MinConnectivity;
extern llvm::cl::opt<double> MinCoverage;
extern llvm::cl::opt<double> MinImplicitCoverage;
} // namespace composition::support
#endif // COMPOSITION_FRAMEWORK_SUPPORT_OPTIONS_HPP
