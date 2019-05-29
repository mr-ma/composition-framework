#include <cassert>
#include <tuple>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/ilp/ILP.hpp>
namespace ilp {
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

  if (m_Name) {
    glp_set_prob_name(m_Problem, m_Name.value().c_str());
  }
  // Set objective direction
  assert(m_Direction != 0);
  glp_set_obj_dir(m_Problem, m_Direction);

  for (auto v : m_Variables) {
    v->apply(*m_Problem);
  }

  // now prepend the required position zero placeholder (any value will do but zero is safe)
  // first create length one vectors using default member construction
  rows.push_back(0);
  cols.push_back(0);
  coefs.push_back(0);
  for (auto c : m_Constraints) {
    const std::vector<std::tuple<int, int, double>> &matrixPartial = c->apply(*m_Problem);
    for (auto[row, col, coef] : matrixPartial) {
      rows.push_back(row);
      cols.push_back(col);
      coefs.push_back(coef);
    }
  }

  // Load the data into the matrix
  int dataSize = static_cast<int>(rows.size()) - 1;
  llvm::dbgs() << "ILP sanity rows:" << dataSize << " columns:" << cols.size() << " coefs:" << coefs.size() << " ia3:"
               << rows[3] << " " << rows[2] << " " << rows[1] << " \n";
  glp_load_matrix(m_Problem, dataSize, &rows[0], &cols[0], &coefs[0]);

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

    for (auto v : m_Variables) {
      delete v;
    }
    m_Variables.clear();

    for (auto c : m_Constraints) {
      delete c;
    }
    m_Constraints.clear();

    rows.clear();
    cols.clear();
    coefs.clear();
  }
}
Constraint *ILP::iff(Variable *a, Variable *b) {
  auto *c = new Constraint{};
  c->setLowerBound(0.0);
  c->setUpperBound(0.0);

  c->add(a, -1.0);
  c->add(b, 1.0);

  return c;
}
Constraint *ILP::XOR(Variable *a, Variable *b) {
  auto *c = new Constraint{};
  c->setLowerBound(0.0);
  c->setUpperBound(1.0);

  c->add(a, 1.0);
  c->add(b, 1.0);
  return c;
}
std::pair<Variable *, Constraint *> ILP::anyOf(const std::vector<Variable *> &vs) {
  auto *v = new Variable();
  v->setLowerBound(0.0);
  v->setUpperBound(1.0);

  auto *c = new Constraint{};
  c->setLowerBound(0.0);
  c->setUpperBound(std::max(size_t(1), vs.size() - 1));

  c->add(v, std::max(size_t(2), vs.size()));

  for (auto var : vs) {
    c->add(var, -1.0);
  }
  return {v, c};
}
Constraint *ILP::nOf(const std::vector<Variable *> &vs, size_t N) {
  auto *c = new Constraint{};
  c->setLowerBound(std::min(vs.size(), N));

  for (auto var : vs) {
    c->add(var, 1.0);
  }

  return c;
}
Constraint *ILP::implication(Variable *a, Variable *b) {
  auto *c = new Constraint{};
  c->setUpperBound(0.0);

  c->add(a, 1.0);
  c->add(b, -1.0);

  return c;
}
Constraint *ILP::maximumNOf(const std::vector<Variable *> &vs, size_t N) {
  auto *c = new Constraint{};
  c->setUpperBound(N);

  for (auto var : vs) {
    c->add(var, 1.0);
  }
  return c;
}
void ILP::writeProblem(const std::string &fileName) {
  llvm::dbgs() << "Writing problem to" << fileName << "\n";
  glp_write_lp(m_Problem, nullptr, fileName.c_str());
}
void ILP::writeSolution(const std::string &fileName) {
  glp_write_mip(m_Problem, fileName.c_str());
}
void ILP::writeSolutionReadable(const std::string &fileName) {
  glp_print_mip(m_Problem, fileName.c_str());
}
double ILP::getValue(Variable *v) {
  return glp_mip_col_val(m_Problem, v->getColumn());
}
}