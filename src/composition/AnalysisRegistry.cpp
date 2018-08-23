#include <composition/AnalysisRegistry.hpp>

namespace composition {

bool AnalysisRegistry::Register(char *ID) {
  // add the pair to the map
  llvm::dbgs() << "Registering analysis pass: " << std::to_string(reinterpret_cast<uintptr_t>(ID)) << "\n";
  RegisteredAnalysis().push_back({ID});
  return true;
}

std::vector<PassRegistrationInfo> &AnalysisRegistry::GetAll() {
  return RegisteredAnalysis();
}

std::vector<PassRegistrationInfo> &AnalysisRegistry::RegisteredAnalysis() {
  static std::vector<PassRegistrationInfo> value = {};
  return value;
}
}