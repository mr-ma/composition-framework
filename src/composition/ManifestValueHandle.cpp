#include <composition/ManifestValueHandle.hpp>
#include <composition/util/functions.hpp>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
using namespace llvm;

namespace composition {
#ifndef NDEBUG
void ManifestValueHandle::deleted() {
  dbgs() << "Value deleted from: " << getPassName() << "\n";
  llvm_unreachable("Value deleted");
}

void ManifestValueHandle::allUsesReplacedWith(Value *value) {
  dbgs() << "Value RAUWED from: " << getPassName() << "\n";
  llvm_unreachable("Value RAUWED");
}

llvm::Value *ManifestValueHandle::operator=(llvm::Value *RHS) {
  return ValueHandleBase::operator=(RHS);
}

llvm::Value *ManifestValueHandle::operator=(const ValueHandleBase &RHS) {
  return ValueHandleBase::operator=(RHS);
}

ManifestValueHandle::operator llvm::Value *() const {
  return getValPtr();
}

bool ManifestValueHandle::pointsToAliveValue() const {
  return llvm::ValueHandleBase::isValid(getValPtr());
}
#else
#endif

}