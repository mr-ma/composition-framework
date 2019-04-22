#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_TRUE_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_TRUE_HPP
#include <composition/graph/constraint/constraint.hpp>
#include <llvm/IR/Value.h>
#include <llvm/IR/ValueHandle.h>

namespace composition::graph::constraint {
/**
 * True constraint which allows optimizing of protections without constraints
 */
class True : public Constraint {
private:
  llvm::Value *target;

public:
  True(std::string info, llvm::Value *target);

  static bool classof(const Constraint *S);

  llvm::Value *getTarget() const;

  bool isValid() override;
};
} // namespace composition::graph::constraint
#endif // COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_TRUE_HPP
