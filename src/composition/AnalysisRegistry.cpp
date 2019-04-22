#include <composition/AnalysisRegistry.hpp>

namespace composition {

bool AnalysisRegistry::Register(PassRegistrationInfo info) {
  llvm::dbgs() << "Registering analysis pass: " << std::addressof(info.ID) << "\n";
  RegisteredAnalysis().push_back(info);
  return true;
}

std::vector<PassRegistrationInfo> &AnalysisRegistry::GetAll() { return RegisteredAnalysis(); }

std::vector<PassRegistrationInfo> &AnalysisRegistry::RegisteredAnalysis() {
  static std::vector<PassRegistrationInfo> value = {};
  return value;
}
} // namespace composition