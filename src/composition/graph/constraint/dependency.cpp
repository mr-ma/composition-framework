#include <composition/graph/constraint/dependency.hpp>

namespace composition::graph::constraint {
Dependency::Dependency(std::string info, llvm::Value* from, llvm::Value* to)
    : Constraint(ConstraintType::CK_DEPENDENCY, GraphType::EDGE, std::move(info)), from(from), to(to) {}

llvm::Value* Dependency::getFrom() const { return from; }

llvm::Value* Dependency::getTo() const { return to; }

bool Dependency::classof(const Constraint* S) { return S->getConstraintType() == ConstraintType::CK_DEPENDENCY; }

bool Dependency::isValid() { return from.pointsToAliveValue() && to.pointsToAliveValue(); }
} // namespace composition::graph::constraint