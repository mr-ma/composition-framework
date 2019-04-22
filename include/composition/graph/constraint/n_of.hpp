#ifndef COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_NOF_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_NOF_HPP
#include <composition/graph/constraint/constraint.hpp>
#include <composition/Manifest.hpp>

namespace composition::graph::constraint {
using composition::Manifest;
/**
 * True constraint which allows optimizing of protections without constraints
 */
class NOf : public Constraint {
private:
  uint64_t N;
  std::vector<Manifest *> manifests;

public:
  NOf(std::string info, uint64_t N, std::vector<Manifest *> manifests);

  static bool classof(const Constraint *S);

  bool isValid() override;

  uint64_t getN() const;

  const std::vector<Manifest *> &getManifests() const;
};
}

#endif //COMPOSITION_FRAMEWORK_GRAPH_CONSTRAINT_NOF_HPP
