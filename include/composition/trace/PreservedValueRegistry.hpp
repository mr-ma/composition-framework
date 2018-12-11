#ifndef COMPOSITION_FRAMEWORK_TRACE_PRESERVEDVALUEREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_TRACE_PRESERVEDVALUEREGISTRY_HPP

#include <composition/trace/TraceableValue.hpp>
#include <llvm/IR/Value.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>

namespace composition::trace {
/**
 * Registry to capture all present/preserved marked values.
 */
// TODO the name of the class is not representing present constraints properly.
class PreservedValueRegistry {
public:
  /**
   * Registers a preserved callback function for the value v
   * @param name the name of the pass
   * @param v the value to protect
   * @param callback the callback
   * @return a struct which defines the necessary information to trigger a callback if a constraint is violated.
   */
  static TraceableCallbackInfo Register(const std::string& name, llvm::Value* v, const PreservedCallback& callback);

  /**
   * Registers a present callback function for the value v
   * @param name the name of the pass
   * @param v the value to protect
   * @param callback the callback
   * @return a struct which defines the necessary information to trigger a callback if a constraint is violated.
   */
  static TraceableCallbackInfo Register(const std::string& name, llvm::Value* v, const PresentCallback& callback);

  /**
   * Registers a present and preserved callback function for the value v
   * @param name the name of the pass
   * @param v the value to protect
   * @param presentCallback the present callback
   * @param preservedCallback the preserved callback
   * @return a struct which defines the necessary information to trigger a callback if a constraint is violated.
   */
  static TraceableCallbackInfo Register(const std::string& name, llvm::Value* v, const PresentCallback& presentCallback,
                                        const PreservedCallback& preservedCallback);

  /**
   * Clears the registered values from the registry.
   */
  static void Clear();

protected:
  static TraceableValueState& TraceableValues();
};
} // namespace composition::trace

#endif // COMPOSITION_FRAMEWORK_TRACE_PRESERVEDVALUEREGISTRY_HPP
