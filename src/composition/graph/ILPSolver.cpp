#include <composition/graph/ILPSolver.hpp>
#include <composition/support/options.hpp>

namespace composition::graph {

ILPSolver::ILPSolver() {
  lp = glp_create_prob();          // creates a problem object
  glp_set_prob_name(lp, "sample"); // assigns a symbolic name to the problem object
  glp_set_obj_dir(lp, GLP_MIN);
}

ILPSolver::~ILPSolver() { destroy(); }

void ILPSolver::destroy() {
  if (lp != nullptr) {
    glp_delete_prob(lp);
  }
  lp = nullptr;
}

void ILPSolver::init(int explicitCov, int implicitCov, double hotness, double hotnessProtectee) {
  // row 1
  EXPLICIT = glp_add_rows(lp, 1);
  glp_set_row_name(lp, EXPLICIT, "explicit");               // assigns name p to first row
  glp_set_row_bnds(lp, EXPLICIT, GLP_LO, explicitCov, 0.0); // 0 < explicit <= inf
  // row 2
  IMPLICIT = glp_add_rows(lp, 1);
  glp_set_row_name(lp, IMPLICIT, "implicit");               // assigns name q to second row
  glp_set_row_bnds(lp, IMPLICIT, GLP_LO, implicitCov, 0.0); // 0 < implicit <= inf
  // row 3
  HOTNESS = glp_add_rows(lp, 1);
  glp_set_row_name(lp, HOTNESS, "hotness");            // assigns name q to second row
  glp_set_row_bnds(lp, HOTNESS, GLP_LO, hotness, 0.0); // 0 < unique <= inf
  // row 4
  HOTNESS_PROTECTEE = glp_add_rows(lp, 1);
  glp_set_row_name(lp, HOTNESS_PROTECTEE, "hotnessProtectee");            // assigns name q to second row
  glp_set_row_bnds(lp, HOTNESS_PROTECTEE, GLP_LO, hotnessProtectee, 0.0); // 0 < unique <= inf
}

void ILPSolver::addManifests(const std::unordered_map<manifest_idx_t, Manifest *> &manifests,
                             std::map<manifest_idx_t, ManifestStats> stats) {
  // COLUMNS
  for (auto&[mIdx, m] : manifests) {
    // column N
    std::ostringstream os;
    os << "m" << m->index;
    auto col = glp_add_cols(lp, 1);
    glp_set_col_name(lp, col, os.str().c_str());          // assigns name m_n to nth column
    glp_set_col_kind(lp, col, GLP_BV);                    // values are binary
    glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0);          // values are binary
    glp_set_obj_coef(lp, col, costFunction(stats[mIdx])); // costs

    colsToM.insert({col, m->index});

    // explicit
    rows.push_back(EXPLICIT);
    cols.push_back(col);
    coeffs.push_back(stats[mIdx].explicitC);

    // manifest has no implicit coverage but edges do
    rows.push_back(IMPLICIT);
    cols.push_back(col);
    // coeffs.push_back(stats[mIdx].implicitC);
    coeffs.push_back(0);

    // hotness
    rows.push_back(HOTNESS);
    cols.push_back(col);
    coeffs.push_back(stats[mIdx].hotness);

    // hotnessProtectee
    rows.push_back(HOTNESS_PROTECTEE);
    cols.push_back(col);
    coeffs.push_back(stats[mIdx].hotnessProtectee);
  }
}

void ILPSolver::addDependencies(const std::set<std::pair<manifest_idx_t, manifest_idx_t>> &dependencies) {
  // Add dependencies
  for (auto &&pair : dependencies) {
    dependency(pair);
  }
}

void ILPSolver::addConflicts(const std::set<std::pair<manifest_idx_t, manifest_idx_t>> &conflicts) {
  // Add conflicts
  for (auto &&pair : conflicts) {
    conflict(pair);
  }
}

void ILPSolver::addCycles(const std::set<std::set<manifest_idx_t>> &cycles) {
  // Add cycles
  for (auto &&c : cycles) {
    cycle(c);
  }
}

void ILPSolver::addConnectivity(const std::set<std::set<manifest_idx_t>> &connectivities) {
  // Add connectivity
  for (auto &&c : connectivities) {
    connectivity(c, DesiredConnectivity);
  }
}

void ILPSolver::addBlockConnectivity(const std::set<std::set<manifest_idx_t>> &blockConnectivities) {
  // Add  blockConnectivity
  for (auto &&c : blockConnectivities) {
    blockConnectivity(c, DesiredBlockConnectivity);
  }
}

std::pair<std::set<manifest_idx_t>, std::set<manifest_idx_t>> ILPSolver::run() {
  int dataSize = static_cast<int>(rows.size());
  // now prepend the required position zero placeholder (any value will do but zero is safe)
  // first create length one vectors using default member construction
  std::vector<int> iav(1, 0);
  std::vector<int> jav(1, 0);
  std::vector<double> arv(1, 0);

  // then concatenate these with the original data vectors
  iav.insert(iav.end(), rows.begin(), rows.end());
  jav.insert(jav.end(), cols.begin(), cols.end());
  arv.insert(arv.end(), coeffs.begin(), coeffs.end());

  glp_load_matrix(lp, dataSize, &iav[0], &jav[0], &arv[0]); // calls the routine glp_load_matrix

  if (!composition::support::ILPProblem.empty()) {
    glp_write_lp(lp, nullptr, composition::support::ILPProblem.getValue().c_str());
  }

  glp_simplex(lp, nullptr); // calls the routine glp_simplex to solve LP problem
  glp_intopt(lp, nullptr);
  auto total_cost = glp_mip_obj_val(lp);

  if (!composition::support::ILPSolution.empty()) {
    glp_print_mip(lp, composition::support::ILPSolution.getValue().c_str());
  }

  std::set<manifest_idx_t> acceptedManifests{};
  for (auto&[col, mIdx] : colsToM) {
    if (glp_mip_col_val(lp, col) == 1) {
      acceptedManifests.insert(mIdx);
      // TODO: calculate implicit coverage based on the accepted edges
    }
  }

  std::set<manifest_idx_t> acceptedEdges{};
  for (auto&[col, eIdx] : colsToE) {
    if (glp_mip_col_val(lp, col) == 1) {
      acceptedEdges.insert(eIdx);
    }
  }

  auto explicitInstructionCoverage = (uint64_t) glp_mip_row_val(lp, EXPLICIT);
  auto implicitInstructionCoverage = (uint64_t) glp_mip_row_val(lp, IMPLICIT);
  llvm::dbgs() << "explicit instruction coverage:" << explicitInstructionCoverage << " implicit instruction coverage:"
               << implicitInstructionCoverage << "\n";
  return {acceptedManifests, acceptedEdges};
}

void ILPSolver::conflict(std::pair<manifest_idx_t, manifest_idx_t> pair) {
  // m1 and m2 conflict; m1 + m2 <= 1
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_UP, 0.0, 1.0);
  std::ostringstream os;
  os << "conflict_" << pair.first << "_" << pair.second;
  glp_set_row_name(lp, row, os.str().c_str());

  rows.push_back(row);
  cols.push_back(colsToM.right.at(pair.first));
  coeffs.push_back(1.0);

  rows.push_back(row);
  cols.push_back(colsToM.right.at(pair.second));
  coeffs.push_back(1.0);
}

void ILPSolver::dependency(std::pair<manifest_idx_t, manifest_idx_t> pair) {
  // m1 depends on m2; m1 <= m2; m1 - m2 <= 0
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_UP, 0.0, 0.0);
  std::ostringstream os;
  os << "dependency_" << pair.first << "_" << pair.second;
  glp_set_row_name(lp, row, os.str().c_str());

  rows.push_back(row);
  cols.push_back(colsToM.right.at(pair.first));
  coeffs.push_back(1.0);

  rows.push_back(row);
  cols.push_back(colsToM.right.at(pair.second));
  coeffs.push_back(-1.0);
}

void ILPSolver::cycle(const std::set<manifest_idx_t> &ms) {
  // m1..mN form a cycle; m1+m2+..+mN <= N-1
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_UP, 0.0, ms.size() - 1);
  std::ostringstream os;
  os << "cycle_" << cycleCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto &idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

void ILPSolver::connectivity(const std::set<manifest_idx_t> &ms, size_t targetConnectivity) {
  // m1..mN protect an Instruction; m1+m2+..+mN >= min(N, targetConnectivity)
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_LO, std::min(ms.size(), targetConnectivity), 0.0);
  std::ostringstream os;
  os << "connectivity_" << connectivityCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto &idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

void ILPSolver::blockConnectivity(const std::set<manifest_idx_t> &ms, size_t targetBlockConnectivity) {
  // m1..mN protect a BasicBlock; m1+m2+..+mN >= min(N, targetBlockConnectivity)
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_LO, std::min(ms.size(), targetBlockConnectivity), 0.0);
  std::ostringstream os;
  os << "block_connectivity_" << blockConnectivityCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto &idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

} // namespace composition::graph
