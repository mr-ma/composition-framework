#include <cassert>
#include <composition/ilp/Constraint.hpp>

std::vector<std::tuple<int /*row*/, int /*col*/, double /*coef*/>> Constraint::apply(glp_prob &m_Problem) {
  m_Row = glp_add_rows(&m_Problem, 1);

  if (m_Name) {
    glp_set_row_name(&m_Problem, m_Row, m_Name.value().c_str());
  }

  glp_set_row_bnds(&m_Problem, m_Row, getBoundType(), m_LowerBound.value_or(0.0), m_UpperBound.value_or(0.0));

  assert(m_Variables.size() == m_Coefficients.size());
  std::vector<std::tuple<int, int, double>> matrix{};
  for (size_t i = 0; i < m_Variables.size(); i++) {
    matrix.emplace_back(m_Row, m_Variables.at(i)->getColumn(), m_Coefficients.at(i));
  }
  return matrix;
}

int Constraint::getBoundType() const {
  if (!m_LowerBound && m_UpperBound) {
    return GLP_UP;
  } else if (m_LowerBound && !m_UpperBound) {
    return GLP_LO;
  } else if (m_LowerBound && m_UpperBound) {
    if (m_LowerBound == m_UpperBound) {
      return GLP_FX;
    } else {
      return GLP_DB;
    }
  }
  return GLP_FR;
}