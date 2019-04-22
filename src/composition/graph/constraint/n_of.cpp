#include <utility>

#include <composition/graph/constraint/n_of.hpp>
#include <composition/graph/constraint/constraint.hpp>

namespace composition::graph::constraint {
NOf::NOf(std::string info, uint64_t N, std::vector<Manifest *> manifests)
    : Constraint(ConstraintType::CK_N_OF, std::move(info)), N(N), manifests(std::move(manifests)) {}

bool NOf::classof(const Constraint *S) { return S->getConstraintType() == ConstraintType::CK_N_OF; }

bool NOf::isValid() { return true; }

uint64_t NOf::getN() const {
  return N;
}

const std::vector<Manifest *> &NOf::getManifests() const {
  return manifests;
}

} // namespace composition::graph::constraint