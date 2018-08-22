#ifndef COMPOSITION_FRAMEWORK_OPTIONS_HPP
#define COMPOSITION_FRAMEWORK_OPTIONS_HPP

#include <llvm/Support/CommandLine.h>

namespace composition {
extern llvm::cl::opt<bool> DumpGraphs;
extern llvm::cl::opt<std::string> WeightConfig;
}
#endif //COMPOSITION_FRAMEWORK_OPTIONS_HPP
