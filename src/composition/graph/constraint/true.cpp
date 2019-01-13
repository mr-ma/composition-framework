#include <composition/graph/constraint/true.hpp>

namespace composition::graph::constraint {
True::True(std::string info, llvm::Value* target)
    : Constraint(ConstraintType::CK_TRUE, std::move(info)), target(target) {}

llvm::Value* True::getTarget() const { return target; }

bool True::classof(const Constraint* S) { return S->getConstraintType() == ConstraintType::CK_TRUE; }

bool True::isValid() { return true; }

} // namespace composition::graph::constraint