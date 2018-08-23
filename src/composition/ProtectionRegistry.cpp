#include <composition/ProtectionRegistry.hpp>

using namespace llvm;

namespace composition {
bool ProtectionRegistry::Register(char *ID) {
  // add the pair to the map
  dbgs() << "Registering post protection pass: " << std::to_string(reinterpret_cast<uintptr_t>(ID)) << "\n";
  RegisteredProtections()->push_back(ID);
  return true;
}

std::vector<char *> ProtectionRegistry::GetAll() {
  return *RegisteredProtections();
}

std::vector<char *> *ProtectionRegistry::RegisteredProtections() {
  static std::vector<char *> value = {};
  return &value;
}
}