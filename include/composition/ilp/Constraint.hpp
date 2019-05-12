#ifndef COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_CONSTRAINT_HPP
#define COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_CONSTRAINT_HPP

#include <string>
#include <optional>
#include <vector>
#include <tuple>
#include <glpk.h>
#include <composition/ilp/Variable.hpp>

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
};

#endif //COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_CONSTRAINT_HPP
