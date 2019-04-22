#include <composition/graph/constraint/preserved.hpp>

namespace composition::graph::constraint {
Preserved::Preserved(std::string info, llvm::Value *target, bool inverse)
    : Constraint(ConstraintType::CK_PRESERVED, std::move(info)), target(target), inverse(inverse) {}

llvm::Value *Preserved::getTarget() const { return target; }

bool Preserved::isInverse() const { return inverse; }

bool Preserved::classof(const Constraint *S) { return S->getConstraintType() == ConstraintType::CK_PRESERVED; }

bool Preserved::isValid() { return target.pointsToAliveValue(); }
} // namespace composition::graph::constraint