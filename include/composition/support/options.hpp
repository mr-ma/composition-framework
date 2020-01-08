#ifndef COMPOSITION_FRAMEWORK_SUPPORT_OPTIONS_HPP
#define COMPOSITION_FRAMEWORK_SUPPORT_OPTIONS_HPP

#include <composition/Stats.hpp>
#include <llvm/Support/CommandLine.h>

namespace composition::support {
/*
 * List of global variables which can be used from anywhere in the framework
 */
extern Stats cStats;
extern llvm::cl::opt<std::string> DumpGraphs;
extern llvm::cl::opt<bool> AddCFG;
extern llvm::cl::opt<std::string> WeightConfig;
extern llvm::cl::opt<std::string> DumpStats;
extern llvm::cl::opt<std::string> UseStrategy;
extern llvm::cl::opt<std::string> PatchInfo;
extern llvm::cl::opt<std::string> ILPProblem;
extern llvm::cl::opt<std::string> ILPSolution;
extern llvm::cl::opt<std::string> ILPSolutionReadable;
/*
 * List of global variables which can be used to control the ILP
 */
extern llvm::cl::opt<int> ILPExplicitBound;
extern llvm::cl::opt<int> ILPImplicitBound;
extern llvm::cl::opt<double> ILPConnectivityBound;
extern llvm::cl::opt<double> ILPBlockConnectivityBound;
extern llvm::cl::opt<double> ILPOverheadBound;
extern llvm::cl::opt<std::string> ILPObjective;


extern llvm::cl::opt<bool> ExperimentalSCIPSolver;
extern llvm::cl::opt<bool> ExperimentalNetworkX;
extern llvm::cl::opt<std::string> ExperimentalNetworkXEdgeFile;
extern llvm::cl::opt<std::string> ExperimentalNetworkXCycleFile;

} // namespace composition::support
#endif // COMPOSITION_FRAMEWORK_SUPPORT_OPTIONS_HPP
