#ifndef COMPOSITION_FRAMEWORK_SUPPORT_MANIFESTVALUEHANDLE_HPP
#define COMPOSITION_FRAMEWORK_SUPPORT_MANIFESTVALUEHANDLE_HPP

#include <llvm/IR/ValueMap.h>

namespace composition::support {
/**
 * A wrapper around CallbackVH (debug) or WeakTrackingVH (release)
 * In a debug build this class provides additional tracking information and triggers LLVM errors if constraints are
 * violated.
 */
#ifndef NDEBUG
class ManifestValueHandle : public llvm::CallbackVH {
public:
  using llvm::CallbackVH::CallbackVH;
  void deleted() override;

  void allUsesReplacedWith(llvm::Value* value) override;

  bool pointsToAliveValue() const;

  explicit operator llvm::Value*() const;
};
#else
class ManifestValueHandle : public llvm::WeakTrackingVH {
public:
  using llvm::WeakTrackingVH::WeakTrackingVH;
};
#endif
} // namespace composition::support

#endif // COMPOSITION_FRAMEWORK_SUPPORT_MANIFESTVALUEHANDLE_HPP
