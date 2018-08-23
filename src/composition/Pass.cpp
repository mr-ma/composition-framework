#include <composition/Pass.hpp>

namespace composition {
bool Pass::IsRegistered() const {
  return this->IsRegistered_;
}

Pass::Pass(bool isRegistered) : IsRegistered_(isRegistered) {}

Pass::Pass() : Pass(false) {}
}