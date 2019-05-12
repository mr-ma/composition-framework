#ifndef COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_ILP_HPP
#define COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_ILP_HPP

#include <vector>
#include <string>
#include <glpk.h>

#include <composition/ilp/Variable.hpp>
#include <composition/ilp/Constraint.hpp>
namespace ilp {
class ILP {
public:
  enum Direction {
    MINIMIZE = GLP_MIN,
    MAXIMIZE = GLP_MAX
  };

private:
  std::optional<std::string> m_Name{};
  std::vector<Variable *> m_Variables{};
  std::vector<Constraint *> m_Constraints{};
  Direction m_Direction{};

  glp_prob *m_Problem = nullptr;
  std::vector<int> rows{};
  std::vector<int> cols{};
  std::vector<double> coefs{};
public:
  void addVariable(Variable *v);
  void addConstraint(Constraint *c);
  void setName(const std::string &n);
  void setDirection(Direction d);
  double getValue(Variable* v);

  void build();
  void run();
  void destroy();

  void writeProblem(const std::string &fileName);
  void writeSolution(const std::string &fileName);
  void writeSolutionReadable(const std::string &fileName);

  /**
   * A <=> B
   * @param a a Variable
   * @param b a Variable
   * @return the resulting constraint
   */
  static Constraint *iff(Variable *a, Variable *b);

  /**
   * True, if any variable of vs is true
   * @param vs a vector of Variables
   * @return the resulting variable and constraint
   */
  static std::pair<Variable *, Constraint *> anyOf(const std::vector<Variable *> &vs);

  /**
   * N variables of vs must be true
   * @param vs a vector of Variables
   * @param N the lowerbound
   * @return the resulting constraint
   */
  static Constraint *nOf(const std::vector<Variable *> &vs, size_t N);

  /**
   * A implies B
   * @param a
   * @param b
   * @return the resulting constraint
   */
  static Constraint *implication(Variable *a, Variable *b);

  /**
   * At most N of vs can be true
   * @param vs a vector of Variables
   * @param N the upperbound
   * @return the resulting constraint
   */
  static Constraint *maximumNOf(const std::vector<Variable *> &vs, size_t N);
  /**
   * A Xor B
   * @param a a Variable
   * @param b a Variable
   * @return the resulting constraint
   */
  static Constraint *XOR(Variable *a, Variable *b);
};
}

#endif //COMPOSITION_FRAMEWORK_INCLUDE_COMPOSITION_ILP_ILP_HPP
