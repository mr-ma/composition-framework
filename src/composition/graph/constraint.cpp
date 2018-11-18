#include <composition/graph/constraint.hpp>

namespace composition::graph {
Constraint::Constraint(Constraint::ConstraintType constraintType, Constraint::GraphType graphType, std::string info) :
    constraintType(constraintType),
    graphType(graphType),
    info(std::move(info)) {
}

Constraint::ConstraintType Constraint::getConstraintType() const {
  return constraintType;
}

Constraint::GraphType Constraint::getGraphType() const {
  return graphType;
}

std::string Constraint::getInfo() const {
  return info;
}

}