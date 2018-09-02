#ifndef COMPOSITION_FRAMEWORK_OPTIONS_HPP
#define COMPOSITION_FRAMEWORK_OPTIONS_HPP

#include <llvm/Support/CommandLine.h>

namespace composition {
extern llvm::cl::opt<bool> DumpGraphs;
extern llvm::cl::opt<bool> AddCFG;
extern llvm::cl::opt<std::string> WeightConfig;
extern llvm::cl::opt<std::string> DumpStats;
extern llvm::cl::opt<std::string> UseStrategy;
}
#endif //COMPOSITION_FRAMEWORK_OPTIONS_HPP
