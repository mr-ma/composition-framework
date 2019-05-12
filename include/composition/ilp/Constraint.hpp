#ifndef COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_CONSTRAINT_HPP
#define COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_CONSTRAINT_HPP

#include <string>
#include <optional>
#include <vector>
#include <tuple>
#include <glpk.h>
#include <composition/ilp/Variable.hpp>

namespace ilp {
class Constraint {
private:
  std::optional<std::string> m_Name;
  std::optional<double> m_LowerBound;
  std::optional<double> m_UpperBound;

  std::vector<Variable *> m_Variables{};
  std::vector<double> m_Coefficients{};

  int m_Row{};

  int getBoundType() const;
public:
  std::vector<std::tuple<int, int, double>> apply(glp_prob &m_Problem);
  void add(Variable *v, double coef);
  void setName(const std::optional<std::string> &MName);
  void setLowerBound(const std::optional<double> &MLowerBound);
  void setUpperBound(const std::optional<double> &MUpperBound);

};
}

#endif //COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_CONSTRAINT_HPP
