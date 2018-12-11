#include <composition/support/ManifestValueHandle.hpp>
#include <composition/util/functions.hpp>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
using namespace llvm;

namespace composition::support {
#ifndef NDEBUG
using util::getPassName;

void ManifestValueHandle::deleted() {
  dbgs() << "Value deleted from: " << getPassName() << "\n";
  this->RemoveFromUseList();
  this->clearValPtr();
}

void ManifestValueHandle::allUsesReplacedWith(Value* value) {
  dbgs() << "Value RAUWED from: " << getPassName() << "\n";
}

ManifestValueHandle::operator llvm::Value*() const { return getValPtr(); }

bool ManifestValueHandle::pointsToAliveValue() const { return llvm::ValueHandleBase::isValid(getValPtr()); }

#else
#endif

} // namespace composition::support