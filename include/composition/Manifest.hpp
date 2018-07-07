#ifndef COMPOSITION_FRAMEWORK_MANIFEST_HPP
#define COMPOSITION_FRAMEWORK_MANIFEST_HPP
#include <utility>
#include <cstdint>
#include <llvm/IR/Value.h>
#include <composition/graph/vertex.hpp>
#include <composition/graph/constraint.hpp>

namespace composition {

class Manifest;

typedef std::function<void(Manifest)> PatchFunction;

struct Manifest {
public:
  unsigned long idx{};
  std::string name;
  PatchFunction patchFunction;
  std::vector<std::shared_ptr<Constraint>> constraints;
  bool postPatching{};

  Manifest(std::string name, PatchFunction patchFunction, std::vector<std::shared_ptr<Constraint>> constraints)
      : name(std::move(name)), patchFunction(std::move(patchFunction)), constraints(std::move(constraints)) {
  }

  Manifest(const std::string &name,
           const PatchFunction &patchFunction,
           const std::vector<std::shared_ptr<Constraint>> &constraints,
           bool postPatching) :
      Manifest(name, patchFunction, constraints) {
    this->postPatching = postPatching;
  }
  //TODO metadata if postpatching is needed - and its direction (from, to, both)

  bool operator==(const Manifest &other) const {
    return (idx == other.idx);
  }

  bool operator<(const Manifest &other) const {
    return (idx < other.idx);
  }
};

}

namespace std {
template<>
struct hash<composition::Manifest> {
  size_t operator()(const composition::Manifest &pt) const {
    return pt.idx;
  }
};
}

#endif //COMPOSITION_FRAMEWORK_MANIFEST_HPP
