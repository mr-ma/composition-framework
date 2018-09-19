#include <composition/Pass.hpp>

namespace composition {
bool Pass::IsRegistered() const {
  return this->IsRegistered_;
}

Pass::Pass(char &pid, bool isRegistered) : ModulePass(pid), IsRegistered_(isRegistered) {}
}