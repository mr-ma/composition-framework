#ifndef COMPOSITION_FRAMEWORK_SUPPORT_ANALYSIS_HPP
#define COMPOSITION_FRAMEWORK_SUPPORT_ANALYSIS_HPP

#include <string>
#include <composition/support/Pass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/Manifest.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/vertex.hpp>
#include <composition/trace/PreservedValueRegistry.hpp>

namespace composition::support {

/**
 * CRTP class to register a Pass with the composition framework
 * @tparam T class to register
 */
template<typename T>
class ComposableAnalysis : public Pass {
public:
  /**
   * Adds a manifest
   * @param m the manifest
   */
  void addProtection(Manifest *m) {
    ManifestRegistry::Add(m);
  }

  /**
   * Marks `value` as preserved. The callback allows to define a custom function call.
   * @param name of the pass
   * @param value marked as preserved
   * @param callback called if preserved is violated (`value` is changed in the program).
   */
  void addPreserved(const std::string &name, llvm::Value *value, const trace::PreservedCallback &callback) {
    trace::PreservedValueRegistry::Register(name, value, callback);
  }

  /**
   * Marks `value` as preserved. The callback allows to define a custom function call.
   * @param name of the pass
   * @param value marked as present
   * @param callback called if present is violated (`value` is removed from the program).
   */
  void addPresent(const std::string &name, llvm::Value *value, const trace::PresentCallback &callback) {
    trace::PreservedValueRegistry::Register(name, value, callback);
  }

protected:
  // to determine if the class is registered
  const static bool IsRegistered_;

  ComposableAnalysis() : Pass(T::ID, IsRegistered_) {}
};

/**
 * attempt to initialise the IsRegistered variable of derived classes whilst registering them to the factory
 * @tparam T class to register
 */
template<typename T>
const bool ComposableAnalysis<T>::IsRegistered_ = AnalysisRegistry::Register({&T::ID});
}
#endif //COMPOSITION_FRAMEWORK_SUPPORT_ANALYSIS_HPP
