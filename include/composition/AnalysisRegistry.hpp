#ifndef COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP

#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <map>
#include <string>
#include <vector>

namespace composition {
/**
 * `PassRegistrationInfo` stores the unique `ID` of a pass.
 */
struct PassRegistrationInfo {
  char *ID;
};

/**
 * `AnalysisRegistry` stores registered passes using the unique `PassRegistrationInfo` of a pass.
 */
class AnalysisRegistry {
public:
  /**
   * Registers the pass with the given `info`.
   * @param info of the pass
   * @return true if the registration succeeded
   */
  static bool Register(PassRegistrationInfo info);

  /**
   * Retrieves all registered passes and returns them.
   * @return a vector of
   */
  static std::vector<PassRegistrationInfo> &GetAll();

protected:
  // TODO: This currently initializes a static vector in the function. Possibly there's a better way to do this.
  static std::vector<PassRegistrationInfo> &RegisteredAnalysis();
};
} // namespace composition
#endif // COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
