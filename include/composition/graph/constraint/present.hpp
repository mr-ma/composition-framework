#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_PRESENT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_PRESENT_HPP
#include <composition/graph/constraint/bitmask.hpp>
#include <composition/graph/constraint/constraint.hpp>
#include <llvm/IR/Value.h>
#include <llvm/IR/ValueHandle.h>

namespace composition::graph::constraint {
/**
 * PresentConstraint enum which represents all possible states
 */
enum class PresentConstraint : unsigned long {
  NONE = 0x00,
  PRESENT = 0x01,
  NOT_PRESENT = 0x02,
  CONFLICT = static_cast<unsigned long>(PRESENT | NOT_PRESENT),
};
ENABLE_BITMASK_OPERATORS(PresentConstraint)

/**
 * Present constraint which signals that `target` is present/not present
 */
class Present : public Constraint {
private:
  llvm::WeakTrackingVH target;
  bool inverse;

public:
  Present(std::string info, llvm::Value *target, bool inverse = false);

  llvm::Value *getTarget() const;

  bool isInverse() const;

  static bool classof(const Constraint *S);

  bool isValid() override;
};
} // namespace composition::graph::constraint
#endif // COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_PRESENT_HPP
