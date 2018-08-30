#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_HPP
#include <llvm/IR/Value.h>
#include <llvm/Support/Casting.h>
#include <composition/util/enums.hpp>

namespace composition {

enum class PresentConstraint : unsigned long {
  NONE = 0x00,
  PRESENT = 0x01,
  NOT_PRESENT = 0x02,
  CONFLICT = static_cast<unsigned long>(PRESENT | NOT_PRESENT),
};

enum class PreservedConstraint : unsigned long {
  NONE = 0x00,
  PRESERVED = 0x01,
  NOT_PRESERVED = 0x02,
  CONFLICT = static_cast<unsigned long>(PRESERVED | NOT_PRESERVED),
};
ENABLE_BITMASK_OPERATORS(PresentConstraint);
ENABLE_BITMASK_OPERATORS(PreservedConstraint);
class Constraint {
public:
  enum class ConstraintKind {
    CK_DEPENDENCY,
    CK_PRESENT,
    CK_PRESERVED
  };

  enum class ConstraintType {
    VERTEX,
    EDGE
  };
private:
  const ConstraintKind Kind;
  const ConstraintType Type;
  const std::string Info;
public:
  Constraint(ConstraintKind Kind, ConstraintType Type, std::string Info);

  Constraint(const Constraint &) = delete;

  ConstraintKind getKind() const;

  ConstraintType getType() const;

  std::string getInfo() const;
};

class Dependency : public Constraint {
private:
  llvm::Value *from;
  llvm::Value *to;
  bool weak;
public:
  Dependency(std::string info, llvm::Value *from, llvm::Value *to, bool weak = false);

  llvm::Value *getFrom() const;

  llvm::Value *getTo() const;

  static bool classof(const Constraint *S);
};

class Present : public Constraint {
private:
  llvm::Value *target;
  bool inverse;
public:
  Present(std::string info, llvm::Value *target, bool inverse = false);

  llvm::Value *getTarget() const;

  bool isInverse() const;

  static bool classof(const Constraint *S);
};

class Preserved : public Constraint {
private:
  llvm::Value *target;
  bool inverse;
public:
  Preserved(std::string info, llvm::Value *target, bool inverse = false);

  llvm::Value *getTarget() const;

  bool isInverse() const;

  static bool classof(const Constraint *S);
};

}
#endif //COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_HPP
