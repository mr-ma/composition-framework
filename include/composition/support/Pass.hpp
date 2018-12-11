#ifndef COMPOSITION_FRAMEWORK_SUPPORT_PASS_HPP
#define COMPOSITION_FRAMEWORK_SUPPORT_PASS_HPP

#include <llvm/Pass.h>

namespace composition::support {

/**
 * Pass structure to define functions for the interaction with the framework.
 */
class Pass : public llvm::ModulePass {
public:
  explicit Pass(char& pid, bool isRegistered);

  ~Pass() override = default;

  /**
   * Helper to identifiy if this analysis pass is registered to the framework.
   * @return true if registered, false otherwise.
   */
  bool IsRegistered() const;

  /**
   * This function is called shortly before the composition framework terminates.
   * It allows to run cleanup/finalize tasks in the registered analyses.
   */
  virtual void finalizeComposition(){};

private:
  const bool IsRegistered_;
};
} // namespace composition::support
#endif // COMPOSITION_FRAMEWORK_SUPPORT_PASS_HPP
