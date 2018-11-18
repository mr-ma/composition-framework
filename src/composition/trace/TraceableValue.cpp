#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/PassSupport.h>
#include <composition/trace/TraceableValue.hpp>
#include <composition/util/functions.hpp>

using namespace llvm;
namespace composition::trace {
using util::getPassName;

template<typename ExtraDataT>
void TraceableValueState::Config::onRAUW(const ExtraDataT &, llvm::Value *oldValue, llvm::Value *newValue) {
  std::vector<std::pair<std::string, PreservedCallback>> callbacks;
  for (auto[it, it_end] = TraceableInfoMap.equal_range(oldValue); it != it_end; ++it) {
    if (it->second.preservedCallback == nullptr) {
      callbacks.emplace_back(it->second.pass, it->second.preservedCallback);
    }
  }
  if (callbacks.empty()) {
    return;
  }

  dbgs() << "RAUWed value but should be preserved\n";
  dbgs() << "Old: ";
  oldValue->print(dbgs(), true);
  dbgs() << " New: ";
  newValue->print(dbgs(), true);
  dbgs() << "\n";
  dbgs() << "Value was deleted by pass: " << getPassName() << "\n";

  for (auto &c : callbacks) {
    dbgs() << "Value was changed by pass: " << getPassName() << "\n";
    c.second(getPassName(), oldValue, newValue);
  }
}

template<typename ExtraDataT>
void TraceableValueState::Config::onDelete(const ExtraDataT &, llvm::Value *oldValue) {
  std::vector<std::pair<std::string, PresentCallback>> callbacks;
  for (auto[it, it_end] = TraceableInfoMap.equal_range(oldValue); it != it_end; ++it) {
    if (it->second.presentCallback == nullptr) {
      callbacks.emplace_back(it->second.pass, it->second.presentCallback);
    }
  }
  if (callbacks.empty()) {
    return;
  }

  dbgs() << "Deleted value but should be preserved\n";
  dbgs() << "Old: ";
  oldValue->print(dbgs(), true);
  dbgs() << "\n";
  dbgs() << "Value was deleted by pass: " << getPassName() << "\n";

  for (auto &c : callbacks) {
    dbgs() << "Value was added by pass: " << c.first << "\n";
    c.second(getPassName(), oldValue);
  }
}

void TraceableValueState::clear() {
  GlobalNumbers.clear();
  TraceableInfoMap.clear();
}

void TraceableValueState::erase(llvm::Value *v) {
  GlobalNumbers.erase(v);
  TraceableInfoMap.erase(v);
}

uint64_t TraceableValueState::getNumber(llvm::Value *v, TraceableCallbackInfo info) {
  auto[MapIter, Inserted] = GlobalNumbers.insert({v, NextNumber});
  if (Inserted)
    NextNumber++;

  TraceableInfoMap.insert({v, info});
  return MapIter->second;
}

std::multimap<llvm::Value *, TraceableCallbackInfo> TraceableValueState::TraceableInfoMap = {};
}