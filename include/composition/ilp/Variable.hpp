#ifndef COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_VARIABLE_HPP
#define COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_VARIABLE_HPP

#include <string>
#include <optional>
#include <glpk.h>

class Variable {
private:
  std::optional<std::string> m_Name{};
  double m_ObjCoefficient{};
  std::optional<double> m_LowerBound{};
  std::optional<double> m_UpperBound{};

  int m_Column{};
private:
  int getBoundType() const;
public:
  int getColumn() const;
  void apply(glp_prob &m_Problem);
};

#endif //COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_VARIABLE_HPP
