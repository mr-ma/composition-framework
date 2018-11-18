#ifndef COMPOSITION_FRAMEWORK_GRAPH_PRESERVED_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PRESERVED_HPP
#include <llvm/IR/Value.h>
#include <llvm/IR/ValueHandle.h>
#include <composition/graph/bitmask.hpp>
#include <composition/graph/constraint.hpp>

namespace composition::graph {
/**
 * PreservedConstraint enum which represents all possible states
 */
enum class PreservedConstraint : unsigned long {
  NONE = 0x00,
  PRESERVED = 0x01,
  NOT_PRESERVED = 0x02,
  CONFLICT = static_cast<unsigned long>(PRESERVED | NOT_PRESERVED),
};
ENABLE_BITMASK_OPERATORS(PreservedConstraint);

/**
 * Preserved constraint which signals that `target` is preserved/not preserved
 */
class Preserved : public Constraint {
private:
  llvm::WeakTrackingVH target;
  bool inverse;
public:
  Preserved(std::string info, llvm::Value *target, bool inverse = false);

  llvm::Value *getTarget() const;

  bool isInverse() const;

  static bool classof(const Constraint *S);

  bool isValid() override;
};
}
#endif //COMPOSITION_FRAMEWORK_GRAPH_PRESERVED_HPP
