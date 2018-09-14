#ifndef COMPOSITION_FRAMEWORK_MANIFESTVALUEHANDLE_HPP
#define COMPOSITION_FRAMEWORK_MANIFESTVALUEHANDLE_HPP

#include <llvm/IR/ValueMap.h>

namespace composition {
#ifndef NDEBUG
class ManifestValueHandle : public llvm::CallbackVH {
public:
  using llvm::CallbackVH::CallbackVH;
  void deleted() override;

  void allUsesReplacedWith(llvm::Value *value) override;

  bool pointsToAliveValue() const;

  explicit operator llvm::Value *() const;

  llvm::Value *operator=(llvm::Value *RHS);

  llvm::Value *operator=(const ValueHandleBase &RHS);
};
#else
class ManifestValueHandle : public llvm::WeakTrackingVH {
public:
  using llvm::WeakTrackingVH::WeakTrackingVH;
};
#endif
}

#endif //COMPOSITION_FRAMEWORK_MANIFESTVALUEHANDLE_HPP
