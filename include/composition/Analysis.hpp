#ifndef COMPOSITION_FRAMEWORK_ANALYSIS_HPP
#define COMPOSITION_FRAMEWORK_ANALYSIS_HPP

#include <string>
#include <composition/Pass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/Manifest.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/vertex.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>

namespace composition {
template<typename T>
class ComposableAnalysis : public Pass {
public:
  void addProtection(Manifest m) {
    ManifestRegistry::Add(std::move(m));
  }

  void addPreserved(const std::string &name, llvm::Value *value, const PreservedCallback &callback) {
    PreservedValueRegistry::Register(name, value, callback);
  }

protected:
  // to determine if the class definition is registered
  const static bool IsRegistered_;

  ComposableAnalysis() : Pass(IsRegistered_) {}
};

template<typename T>
// attempt to initialise the IsRegistered variable of derived classes
// whilst registering them to the factory
const bool ComposableAnalysis<T>::IsRegistered_ = AnalysisRegistry::Register(&T::ID, &T::PostPatchingRequired);
}
#endif //COMPOSITION_FRAMEWORK_ANALYSIS_HPP
