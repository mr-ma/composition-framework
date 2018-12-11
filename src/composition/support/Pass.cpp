#include <composition/support/Pass.hpp>

namespace composition::support {
Pass::Pass(char& pid, bool isRegistered) : ModulePass(pid), IsRegistered_(isRegistered) {}

bool Pass::IsRegistered() const { return this->IsRegistered_; }
} // namespace composition::support