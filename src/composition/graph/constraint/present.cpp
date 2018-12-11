#include <composition/graph/constraint/present.hpp>

namespace composition::graph::constraint {
Present::Present(std::string info, llvm::Value* target, bool inverse)
    : Constraint(ConstraintType::CK_PRESENT, GraphType::VERTEX, std::move(info)), target(target), inverse(inverse) {}

llvm::Value* Present::getTarget() const { return target; }

bool Present::isInverse() const { return inverse; }

bool Present::classof(const Constraint* S) { return S->getConstraintType() == ConstraintType::CK_PRESENT; }

bool Present::isValid() { return target.pointsToAliveValue(); }

} // namespace composition::graph::constraint