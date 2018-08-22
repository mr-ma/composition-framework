#include <composition/options.hpp>

using namespace llvm;

namespace composition {
cl::opt<bool> DumpGraphs("cf-dump-graphs", cl::Hidden, cl::desc("Graph files are written to disk"));
}