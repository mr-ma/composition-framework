#ifndef COMPOSITION_FRAMEWORK_PASS_HPP
#define COMPOSITION_FRAMEWORK_PASS_HPP

#include <string>
#include <llvm/Support/ErrorHandling.h>
#include <functional>
#include <llvm/Pass.h>

namespace composition {

class Pass : public llvm::ModulePass {
public:
  explicit Pass(char &pid, bool isRegistered);

  ~Pass() override = default;

  // to determine if this instance class is an instance
  // of a derived class, registered to the factory.
  bool IsRegistered() const;

  virtual void finalizeComposition() {};

private:
  const bool IsRegistered_;
};
}
#endif //COMPOSITION_FRAMEWORK_PASS_HPP
