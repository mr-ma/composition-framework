#include <composition/options.hpp>

using namespace llvm;

namespace composition {
cl::opt<bool> DumpGraphs("cf-dump-graphs", cl::Hidden, cl::desc("Graph files are written to disk"));
cl::opt<bool> AddCFG("cf-add-cfg", cl::Hidden, cl::desc("Adds CFG edges to graphs"));
cl::opt<std::string> WeightConfig("cf-weights", cl::Hidden, cl::desc("Weights to influence the metrics used to decide if a conflict is resolved."));
}