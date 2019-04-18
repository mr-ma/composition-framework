#include <composition/graph/ILPSolver.hpp>

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
  // ROWS
  glp_add_rows(lp, 4); // adds three rows to the problem object
  // row 1
  glp_set_row_name(lp, 1, "explicit");               // assigns name p to first row
  glp_set_row_bnds(lp, 1, GLP_LO, explicitCov, 0.0); // 0 < explicit <= inf
  // row 2
  glp_set_row_name(lp, 2, "implicit");               // assigns name q to second row
  glp_set_row_bnds(lp, 2, GLP_LO, implicitCov, 0.0); // 0 < implicit <= inf
  // row 3
  glp_set_row_name(lp, 3, "hotness");            // assigns name q to second row
  glp_set_row_bnds(lp, 3, GLP_LO, hotness, 0.0); // 0 < unique <= inf
  // row 4
  glp_set_row_name(lp, 4, "hotnessProtectee");            // assigns name q to second row
  glp_set_row_bnds(lp, 4, GLP_LO, hotnessProtectee, 0.0); // 0 < unique <= inf
}

void ILPSolver::addManifests(std::unordered_map<manifest_idx_t, Manifest*> manifests,
                             std::map<manifest_idx_t, ManifestStats> stats) {
  // COLUMNS
  for (auto& [mIdx, m] : manifests) {
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
    rows.push_back(1);
    cols.push_back(col);
    coeffs.push_back(stats[mIdx].explicitC);

    // manifest has no implicit coverage but edges do
    rows.push_back(2);
    cols.push_back(col);
    // coeffs.push_back(stats[mIdx].implicitC);
    coeffs.push_back(0);

    // hotness
    rows.push_back(3);
    cols.push_back(col);
    coeffs.push_back(stats[mIdx].hotness);

    // hotnessProtectee
    rows.push_back(4);
    cols.push_back(col);
    coeffs.push_back(stats[mIdx].hotnessProtectee);
  }
}

void ILPSolver::addDependencies(std::set<std::pair<manifest_idx_t, manifest_idx_t>> dependencies) {
  // Add dependencies
  for (auto&& pair : dependencies) {
    dependency(pair);
  }
}

void ILPSolver::addConflicts(std::set<std::pair<manifest_idx_t, manifest_idx_t>> conflicts) {
  // Add conflicts
  for (auto&& pair : conflicts) {
    conflict(pair);
  }
}

void ILPSolver::addCycles(std::set<std::set<manifest_idx_t>> cycles) {
  // Add cycles
  for (auto&& c : cycles) {
    cycle(c);
  }
}

void ILPSolver::addConnectivity(std::set<std::set<manifest_idx_t>> connectivities) {
  // Add connectivity
  for (auto&& c : connectivities) {
    connectivity(c, DesiredConnectivity);
  }
}

void ILPSolver::addBlockConnectivity(std::set<std::set<manifest_idx_t>> blockConnectivities) {
  // Add  blockConnectivity
  for (auto&& c : blockConnectivities) {
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
  llvm::dbgs() << "Writing prob.glp\n";
  glp_write_lp(lp, nullptr, "prob.glp");

  glp_simplex(lp, nullptr); // calls the routine glp_simplex to solve LP problem
  glp_intopt(lp, nullptr);
  auto total_cost = glp_mip_obj_val(lp);
  glp_print_mip(lp, "sol.glp");

  std::set<manifest_idx_t> acceptedManifests{};
  for (auto& [col, mIdx] : colsToM) {
    if (glp_mip_col_val(lp, col) == 1) {
      acceptedManifests.insert(mIdx);
      // TODO: calculate implicit coverage based on the accepted edges
    }
  }

  std::set<manifest_idx_t> acceptedEdges{};
  for (auto& [col, eIdx] : colsToE) {
    if (glp_mip_col_val(lp, col) == 1) {
      acceptedEdges.insert(eIdx);
    }
  }


  uint64_t explicitInstructionCoverage = glp_mip_row_val(lp,1);
  uint64_t implicitInstructionCoverage = glp_mip_row_val(lp,2); 
  llvm::dbgs()<<"explicit instruction coverage:"<<explicitInstructionCoverage<<" implicit instruction coverage:"<<implicitInstructionCoverage<<"\n";
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

void ILPSolver::cycle(std::set<manifest_idx_t> ms) {
  // m1..mN form a cycle; m1+m2+..+mN <= N-1
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_UP, 0.0, ms.size() - 1);
  std::ostringstream os;
  os << "cycle_" << cycleCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto& idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

void ILPSolver::connectivity(std::set<manifest_idx_t> ms, size_t targetConnectivity) {
  // m1..mN protect an Instruction; m1+m2+..+mN >= min(N, targetConnectivity)
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_LO, std::min(ms.size(), targetConnectivity), 0.0);
  std::ostringstream os;
  os << "connectivity_" << connectivityCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto& idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

void ILPSolver::blockConnectivity(std::set<manifest_idx_t> ms, size_t targetBlockConnectivity) {
  // m1..mN protect a BasicBlock; m1+m2+..+mN >= min(N, targetBlockConnectivity)
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_LO, std::min(ms.size(), targetBlockConnectivity), 0.0);
  std::ostringstream os;
  os << "block_connectivity_" << blockConnectivityCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto& idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

} // namespace composition::graph
