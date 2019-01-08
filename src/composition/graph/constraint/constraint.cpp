#include <composition/graph/constraint/constraint.hpp>

namespace composition::graph::constraint {

constraint_idx_t& operator++(constraint_idx_t& i) {
  using T = typename std::underlying_type<constraint_idx_t>::type;
  auto val = static_cast<T>(i);
  i = constraint_idx_t(++val);
  return i;
}

const constraint_idx_t operator++(constraint_idx_t& i, int) {
  constraint_idx_t res(i);
  ++i;
  return res;
}

Constraint::Constraint(Constraint::ConstraintType constraintType, Constraint::GraphType graphType, std::string info)
    : constraintType(constraintType), graphType(graphType), info(std::move(info)) {}

Constraint::ConstraintType Constraint::getConstraintType() const { return constraintType; }

Constraint::GraphType Constraint::getGraphType() const { return graphType; }

std::string Constraint::getInfo() const { return info; }

} // namespace composition::graph::constraint