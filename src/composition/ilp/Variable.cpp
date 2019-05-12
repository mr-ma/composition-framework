#include <composition/ilp/Variable.hpp>

void Variable::apply(glp_prob &m_Problem) {
  m_Column = glp_add_cols(&m_Problem, 1);
  glp_set_col_kind(&m_Problem, m_Column, GLP_BV);

  if (m_Name) {
    glp_set_col_name(&m_Problem, m_Column, m_Name.value().c_str());
  }

  glp_set_obj_coef(&m_Problem, m_Column, m_ObjCoefficient);
  glp_set_col_bnds(&m_Problem, m_Column, getBoundType(), m_LowerBound.value_or(0.0), m_UpperBound.value_or(0.0));
}
int Variable::getColumn() const {
  return m_Column;
}

int Variable::getBoundType() const {
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
