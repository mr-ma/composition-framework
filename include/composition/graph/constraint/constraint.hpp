#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_CONSTRAINT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_CONSTRAINT_HPP
#include <string>

namespace composition::graph::constraint {
// ConstraintIndex type. TODO C++ does not enforce type safety. Potentially there are ways how type safety can be
// improved.
typedef unsigned long ConstraintIndex;
/**
 * Abstract class Constraint
 */
class Constraint {
public:
  /**
   * Existing types of constraints
   */
  enum class ConstraintType { CK_DEPENDENCY, CK_PRESENT, CK_PRESERVED, CK_TRUE };

  /**
   * Type in graph representation
   */
  enum class GraphType { VERTEX, EDGE };

private:
  /**
   * The type of the constraint to support LLVM's dyn_cast
   */
  const ConstraintType constraintType;
  /**
   * The type of the constraint in a graph. TODO: Currently unused.
   */
  const GraphType graphType;
  /**
   * Additional info about this constraint, e.g., the pass that added it.
   */
  const std::string info;

public:
  Constraint(ConstraintType constraintType, GraphType graphType, std::string info);

  Constraint(const Constraint&) = delete;

  ConstraintType getConstraintType() const;

  GraphType getGraphType() const;

  std::string getInfo() const;

  /**
   * Health-Check of constraint
   */
  virtual bool isValid() = 0;
};
} // namespace composition::graph::constraint
#endif // COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_CONSTRAINT_HPP
