#include <composition/graph/constraint/constraint.hpp>

namespace composition::graph::constraint {

constraint_idx_t& operator++(constraint_idx_t& i) {
  auto val = static_cast<typename std::underlying_type<constraint_idx_t>::type>(i);
  i = constraint_idx_t(++val);
  return i;
}

const constraint_idx_t operator++(constraint_idx_t& i, int) {
  constraint_idx_t res(i);
  ++i;
  return res;
}

Constraint::Constraint(Constraint::ConstraintType constraintType, std::string info)
    : constraintType(constraintType), info(std::move(info)) {}

Constraint::ConstraintType Constraint::getConstraintType() const { return constraintType; }

std::string Constraint::getInfo() const { return info; }

} // namespace composition::graph::constraint