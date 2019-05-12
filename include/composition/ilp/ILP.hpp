#ifndef COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_ILP_HPP
#define COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_ILP_HPP

#include <vector>
#include <string>
#include <glpk.h>

#include <composition/ilp/Variable.hpp>
#include <composition/ilp/Constraint.hpp>

class ILP {
public:
  enum Direction {
    MINIMIZE = GLP_MIN,
    MAXIMIZE = GLP_MAX
  };

private:
  std::string m_Name{};
  std::vector<Variable *> m_Variables{};
  std::vector<Constraint *> m_Constraints{};
  Direction m_Direction{};

  glp_prob *m_Problem = nullptr;
  std::vector<int> rows{};
  std::vector<int> cols{};
  std::vector<double> coefs{};
public:
  inline void addVariable(Variable *v);
  inline void addConstraint(Constraint *c);
  inline std::vector<Variable *> getVariables();
  inline std::vector<Constraint *> getConstraints();
  inline void setName(const std::string &n);
  inline void setDirection(Direction d);

  inline void build();
  inline void run();
  inline void destroy();
};

#endif //COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_ILP_HPP
