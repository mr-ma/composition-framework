#ifndef COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP

#include <string>
#include <map>
#include <vector>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>

namespace composition {
struct PassRegistrationInfo {
  char *ID;
};

class AnalysisRegistry {
public:
  // register a class name with a particular create method
  static bool Register(char *ID);

  static std::vector<PassRegistrationInfo> &GetAll();;
protected:
  // a map to hold a ... mapping between strings and create functions
  static std::vector<PassRegistrationInfo> &RegisteredAnalysis();;
};
}
#endif //COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
