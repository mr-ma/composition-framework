#ifndef COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_VARIABLE_HPP
#define COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_VARIABLE_HPP

#include <string>
#include <optional>
#include <glpk.h>
namespace ilp {
class Variable {
private:
  std::optional<std::string> m_Name{};
  double m_ObjCoefficient{};
  std::optional<double> m_LowerBound{0.0};
  std::optional<double> m_UpperBound{1.0};

  int m_Column{};
private:
  int getBoundType() const;
public:
  int getColumn() const;
  void apply(glp_prob &m_Problem);

  void setName(const std::optional<std::string> &MName);
  const std::optional<std::string> &getName() const;
  void setObjCoefficient(double MObjCoefficient);
  void setLowerBound(const std::optional<double> &MLowerBound);
  void setUpperBound(const std::optional<double> &MUpperBound);
};
}

#endif //COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_VARIABLE_HPP
