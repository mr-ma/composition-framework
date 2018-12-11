#include <composition/graph/constraint/constraint.hpp>

namespace composition::graph::constraint {
Constraint::Constraint(Constraint::ConstraintType constraintType, Constraint::GraphType graphType, std::string info)
    : constraintType(constraintType), graphType(graphType), info(std::move(info)) {}

Constraint::ConstraintType Constraint::getConstraintType() const { return constraintType; }

Constraint::GraphType Constraint::getGraphType() const { return graphType; }

std::string Constraint::getInfo() const { return info; }

} // namespace composition::graph::constraint