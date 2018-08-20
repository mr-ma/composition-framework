#include <utility>

#include <string>
#include <composition/trace/PreservedValueRegistry.hpp>

using namespace llvm;
namespace composition {
TraceableCallbackInfo PreservedValueRegistry::Register(const std::string &name, llvm::Value *v, const PreservedCallback &callback) {
  dbgs() << "Registering preserved value: ";
  v->print(dbgs());
  dbgs() << "\n";

  auto info = TraceableCallbackInfo(name, nullptr, callback);
  TraceableValues().getNumber(v, info);
  return info;
}

TraceableCallbackInfo PreservedValueRegistry::Register(const std::string &name, llvm::Value *v, const PresentCallback &callback) {
  dbgs() << "Registering present value: ";
  v->print(dbgs());
  dbgs() << "\n";

  auto info = TraceableCallbackInfo(name, callback, nullptr);
  TraceableValues().getNumber(v, info);
  return info;
}

TraceableCallbackInfo PreservedValueRegistry::Register(const std::string &name,
                                      llvm::Value *v,
                                      const PresentCallback &presentCallback,
                                      const PreservedCallback &preservedCallback) {

  dbgs() << "Registering present preserved value: ";
  v->print(dbgs());
  dbgs() << "\n";

  auto info = TraceableCallbackInfo(name, presentCallback, preservedCallback);
  TraceableValues().getNumber(v, info);
  return info;
}

void PreservedValueRegistry::Clear() {
  dbgs() << "All values were correctly preserved\n";
  TraceableValues().clear();
}

TraceableValueState &PreservedValueRegistry::TraceableValues() {
  static TraceableValueState value = {};
  return value;
}
}