#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_CONSTRAINT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_CONSTRAINT_HPP
#include <string>

namespace composition::graph::constraint {
// ConstraintIndex type.
enum class constraint_idx_t : uint64_t;
constraint_idx_t &operator++(constraint_idx_t &i);
const constraint_idx_t operator++(constraint_idx_t &i, int);

/**
 * Abstract class Constraint
 */
class Constraint {
public:
  /**
   * Existing types of constraints
   */
  enum class ConstraintType { CK_DEPENDENCY, CK_PRESENT, CK_PRESERVED, CK_TRUE, CK_N_OF };

private:
  /**
   * The type of the constraint to support LLVM's dyn_cast
   */
  const ConstraintType constraintType;
  /**
   * Additional info about this constraint, e.g., the pass that added it.
   */
  const std::string info;

public:
  Constraint(ConstraintType constraintType, std::string info);

  Constraint(const Constraint &) = delete;

  ConstraintType getConstraintType() const;

  std::string getInfo() const;

  /**
   * Health-Check of constraint
   */
  virtual bool isValid() = 0;
};
} // namespace composition::graph::constraint
#endif // COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_CONSTRAINT_HPP
