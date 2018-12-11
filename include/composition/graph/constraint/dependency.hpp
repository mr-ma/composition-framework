#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_DEPENDENCY_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_DEPENDENCY_HPP
#include <composition/graph/constraint/constraint.hpp>
#include <llvm/IR/Value.h>
#include <llvm/IR/ValueHandle.h>

namespace composition::graph::constraint {
/**
 * Dependency constraint which forms a dependency between `from` and `to`.
 */
class Dependency : public Constraint {
private:
  llvm::WeakTrackingVH from;
  llvm::WeakTrackingVH to;

public:
  Dependency(std::string info, llvm::Value* from, llvm::Value* to);

  llvm::Value* getFrom() const;

  llvm::Value* getTo() const;

  static bool classof(const Constraint* S);

  bool isValid() override;
};
} // namespace composition::graph::constraint
#endif // COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_DEPENDENCY_HPP
