#ifndef COMPOSITION_FRAMEWORK_TRACE_TRACEABLEVALUE_HPP
#define COMPOSITION_FRAMEWORK_TRACE_TRACEABLEVALUE_HPP
#include <functional>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/ValueMap.h>
#include <map>
#include <utility>

namespace composition::trace {

/**
 * Function that is called if a preserved constraint is violated
 */
typedef std::function<void(const std::string &, llvm::Value *, llvm::Value *)> PreservedCallback;

/**
 * Function that is called if a present constraint is violated
 */
typedef std::function<void(const std::string &, llvm::Value *)> PresentCallback;

/**
 * Captures the present and preserved callback for a protected value
 */
struct TraceableCallbackInfo {
  std::string pass;
  PresentCallback presentCallback;
  PreservedCallback preservedCallback;

  TraceableCallbackInfo(std::string pass, PresentCallback presentCallback, PreservedCallback preservedCallback)
      : pass(std::move(pass)), presentCallback(std::move(presentCallback)),
        preservedCallback(std::move(preservedCallback)) {}
};

/**
 * Registers the callbacks with LLVMs ValueMap. This calls onRAUW and onDelete if a stored value is changed or removed.
 */
class TraceableValueState {
  struct Config : llvm::ValueMapConfig<llvm::Value *> {
    enum { FollowRAUW = true };

    template<typename ExtraDataT>
    static void onRAUW(const ExtraDataT &data, llvm::Value *oldValue, llvm::Value *newValue);

    template<typename ExtraDataT> static void onDelete(const ExtraDataT &data, llvm::Value *oldValue);
  };

  // Each GlobalValue is mapped to an identifier. The Config ensures when RAUW
  // occurs, the mapping is changed.
  using ValueNumberMap = llvm::ValueMap<llvm::Value *, uint64_t, Config>;
  ValueNumberMap GlobalNumbers;

  static std::multimap<llvm::Value *, TraceableCallbackInfo> TraceableInfoMap;

  // The next unused serial number to assign to a global.
  uint64_t NextNumber = 0;

public:
  TraceableValueState() = default;

  uint64_t getNumber(llvm::Value *v, TraceableCallbackInfo info);

  void erase(llvm::Value *v);

  void clear();
};

} // namespace composition::trace

#endif // COMPOSITION_FRAMEWORK_TRACE_TRACEABLEVALUE_HPP
