#include <cassert>
#include <tuple>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/ilp/ILP.hpp>

void ILP::addVariable(Variable *v) {
  m_Variables.push_back(v);
}
void ILP::addConstraint(Constraint *c) {
  m_Constraints.push_back(c);
}
void ILP::setDirection(ILP::Direction d) {
  m_Direction = d;
}
void ILP::setName(const std::string &n) {
  m_Name = n;
}
void ILP::build() {
  assert(m_Problem == nullptr);
  m_Problem = glp_create_prob();

  assert(!m_Name.empty());
  glp_set_prob_name(m_Problem, m_Name.c_str());

  // Set objective direction
  assert(m_Direction != 0);
  glp_set_obj_dir(m_Problem, m_Direction);

  for (auto v : m_Variables) {
    v->apply(*m_Problem);
  }

  for (auto c : m_Constraints) {
    const std::vector<std::tuple<int, int, double>> &matrixPartial = c->apply(*m_Problem);
    for (auto[row, col, coef] : matrixPartial) {
      rows.push_back(row);
      cols.push_back(col);
      coefs.push_back(coef);
    }
  }
}
void ILP::run() {
  // Parameters
  glp_iocp params{};
  glp_init_iocp(&params);

  params.gmi_cuts = GLP_ON;
  params.br_tech = GLP_BR_PCH;
  params.presolve = GLP_ON;

  // Run ILP solver
  int ecode = glp_intopt(m_Problem, &params);
  int status = glp_mip_status(m_Problem);
  llvm::dbgs() << "MIP exit code: " << ecode << " status code: " << status << "\n";

  // No error occured
  assert(ecode == 0);
  // Solution is INTEGER OPTIMAL
  assert(status == GLP_OPT);
}
void ILP::destroy() {
  if (m_Problem != nullptr) {
    glp_delete_prob(m_Problem);
    m_Problem = nullptr;
  }
}
std::vector<Variable *> ILP::getVariables() {
  return m_Variables;
}
std::vector<Constraint *> ILP::getConstraints() {
  return m_Constraints;
}
