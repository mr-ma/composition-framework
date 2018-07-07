#ifndef COMPOSITION_FRAMEWORK_TRACE_TRACEABLEVALUE_HPP
#define COMPOSITION_FRAMEWORK_TRACE_TRACEABLEVALUE_HPP
#include <map>
#include <utility>
#include <llvm/IR/Value.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/ValueMap.h>

namespace composition {

typedef std::function<void(const std::string &, llvm::Value *, llvm::Value *)> PreservedCallback;

struct TraceableCallbackInfo {
  std::string pass;
  PreservedCallback callback;

  TraceableCallbackInfo(std::string pass, PreservedCallback callback)
      : pass(std::move(pass)), callback(std::move(callback)) {}
};

class TraceableValueState {
  struct Config : llvm::ValueMapConfig<llvm::Value *> {
    enum { FollowRAUW = true };

    template<typename ExtraDataT>
    static void onRAUW(const ExtraDataT &data, llvm::Value *oldValue, llvm::Value *newValue);

    template<typename ExtraDataT>
    static void onDelete(const ExtraDataT &data, llvm::Value *oldValue);
  };

  // Each GlobalValue is mapped to an identifier. The Config ensures when RAUW
  // occurs, the mapping is changed.
  using ValueNumberMap = llvm::ValueMap<llvm::Value *, uint64_t, Config>;
  ValueNumberMap GlobalNumbers;

  static std::map<llvm::Value *, TraceableCallbackInfo> ValueNameMap;

  // The next unused serial number to assign to a global.
  uint64_t NextNumber = 0;

public:
  TraceableValueState() = default;

  uint64_t getNumber(llvm::Value *v, TraceableCallbackInfo info);

  void erase(llvm::Value *v);

  void clear();
};

}

#endif //COMPOSITION_FRAMEWORK_TRACE_TRACEABLEVALUE_HPP
