#include <composition/graph/ILPSolver.hpp>
#include <composition/support/options.hpp>

namespace composition::graph {

ILPSolver::ILPSolver() {
  lp = glp_create_prob();          // creates a problem object
  glp_set_prob_name(lp, "sample"); // assigns a symbolic name to the problem object
}

ILPSolver::~ILPSolver() { destroy(); }

void ILPSolver::destroy() {
  if (lp != nullptr) {
    glp_delete_prob(lp);
  }
  lp = nullptr;
}

void ILPSolver::init(const std::string &objectiveMode, double overheadBound, int explicitBound, int implicitBound,
                     double hotness, double hotnessProtectee) {

  this->setMode(objectiveMode);
  //after setting the mode we can set the objective direction, min or max
  glp_set_obj_dir(lp, get_obj_dir());
  addModeRows(overheadBound, explicitBound, implicitBound, hotness, hotnessProtectee);
}

void ILPSolver::addManifests(const std::unordered_map<manifest_idx_t, Manifest *> &manifests,
                             std::map<manifest_idx_t, ManifestStats> stats) {
  // COLUMNS
  for (auto&[mIdx, m] : manifests) {
    // column N
    std::ostringstream os;
    os << "m" << m->index;
    auto col = glp_add_cols(lp, 1);
    glp_set_col_name(lp, col, os.str().c_str());                  // assigns name m_n to nth column
    glp_set_col_kind(lp, col, GLP_BV);                            // values are binary
    glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0);                  // values are binary
    glp_set_obj_coef(lp, col, get_obj_coef_manifest(costFunction(stats[mIdx]))); // costs

    colsToM.insert({col, m->index});
    // depending on the objective columns need to be added differently
    addModeColumns(col,
                   costFunction(stats[mIdx]) /*overhead*/,
                   0 /*explicit(only instructions)*/,//stats[mIdx].explicitC,
                   0 /*implicit(only edges)*/,
                   stats[mIdx].hotness,
                   stats[mIdx].hotnessProtectee, /*isManifest*/
                   1);
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
    connectivity(c, ILPConnectivityBound);
  }
}

void ILPSolver::addBlockConnectivity(const std::set<std::set<manifest_idx_t>> &blockConnectivities) {
  // Add  blockConnectivity
  for (auto &&c : blockConnectivities) {
    blockConnectivity(c, ILPBlockConnectivityBound);
  }
}

void ILPSolver::addExplicitCoverages(const std::map<llvm::Instruction *, std::set<manifest_idx_t>> &coverage) {
  // Add explicit coverage
  for (auto &[I, c] : coverage) {
    explicitCoverage(I, c);
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

  llvm::dbgs() << "ILP sanity rows:" << dataSize << " columns:" << cols.size() << " coefs:" << coeffs.size() << " ia3:"
               << rows[3] << " " << rows[2] << " " << rows[1] << " \n";
  glp_load_matrix(lp, dataSize, &iav[0], &jav[0], &arv[0]); // calls the routine glp_load_matrix

  // Write problem definition
  if (!composition::support::ILPProblem.empty()) {
    llvm::dbgs() << "Writing problem to" << composition::support::ILPProblem.getValue() << "\n";
    glp_write_lp(lp, nullptr, composition::support::ILPProblem.getValue().c_str());
  }

  glp_iocp params{};
  glp_init_iocp(&params);

  params.gmi_cuts = GLP_ON;
  params.br_tech = GLP_BR_PCH;
  params.presolve = GLP_ON;

  int mip_ecode = glp_intopt(lp, &params);
  int mip_status = glp_mip_status(lp);
  llvm::dbgs() << "MIP exit code: " << mip_ecode << " status code: " << mip_ecode << "\n";
  // No error occured
  assert(mip_ecode == 0);
  // Solution is INTEGER OPTIMAL
  assert(mip_status == GLP_OPT);

  auto objective_result = glp_mip_obj_val(lp);

  // Write machine readable solution
  if (!composition::support::ILPSolution.empty()) {
    glp_write_mip(lp, composition::support::ILPSolution.getValue().c_str());
  }

  // Write human readable solution
  if (!composition::support::ILPSolutionReadable.empty()) {
    glp_print_mip(lp, composition::support::ILPSolutionReadable.getValue().c_str());
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
  printModeILPResults();

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

void ILPSolver::connectivity(const std::set<manifest_idx_t> &ms, double targetConnectivity) {
  // m1..mN protect an Instruction; m1+m2+..+mN >= min(N, targetConnectivity)
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_LO, std::min((double) ms.size(), targetConnectivity), 0.0);
  std::ostringstream os;
  os << "connectivity_" << connectivityCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto &idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

void ILPSolver::blockConnectivity(const std::set<manifest_idx_t> &ms, double targetBlockConnectivity) {
  // m1..mN protect a BasicBlock; m1+m2+..+mN >= min(N, targetBlockConnectivity)
  auto row = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, row, GLP_LO, std::min((double) ms.size(), targetBlockConnectivity), 0.0);
  std::ostringstream os;
  os << "block_connectivity_" << blockConnectivityCount++;
  glp_set_row_name(lp, row, os.str().c_str());

  for (auto &idx : ms) {
    rows.push_back(row);
    cols.push_back(colsToM.right.at(idx));
    coeffs.push_back(1.0);
  }
}

void ILPSolver::explicitCoverage(llvm::Instruction *I, const std::set<manifest_idx_t> &ms) {
  std::ostringstream os;
  os << "i" << instructionCount;

  auto col = glp_add_cols(lp, 1);

  glp_set_col_name(lp, col, os.str().c_str()); // assigns name m_n to nth column
  glp_set_col_kind(lp, col, GLP_BV);                      // values are binary
  glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0);            // values are binary
  glp_set_obj_coef(lp, col, get_obj_coef_explicit(1));

  ItoCols.insert({I, col});
  addModeColumns(col, 0, 1 /*explicit cov of instruction*/, 0, 0, 0, 0);

  // any of m1...mN if c
  auto orRow = glp_add_rows(lp, 1);
  glp_set_row_bnds(lp, orRow, GLP_DB, 0.0, std::max(size_t(1), ms.size() - 1));
  os << "_row";
  glp_set_row_name(lp, orRow, os.str().c_str());

  rows.push_back(orRow);
  cols.push_back(col);
  coeffs.push_back(std::max(size_t(2), ms.size()));

  for (auto m : ms) {
    rows.push_back(orRow);
    cols.push_back(colsToM.right.at(m));
    coeffs.push_back(-1.0);
  }
  instructionCount++;
}

void ILPSolver::addUndoDependencies(const std::unordered_map<manifest_idx_t, Manifest *> &manifests) {
  llvm::dbgs() << "Size: " << ItoCols.size() << "\n";
  for (auto[idx, m] : manifests) {
    for (auto v : m->UndoValues()) {
      if (auto I = llvm::dyn_cast<llvm::Instruction>(v)) {
        // m1 <=> i1
        auto it = ItoCols.find(I);
        if (it == ItoCols.end()) {
          continue;
        }
        auto iCol = it->second;
        auto mCol = colsToM.right.at(idx);

        // m1 <=> i1
        auto row = glp_add_rows(lp, 1);
        glp_set_row_bnds(lp, row, GLP_FX, 0.0, 0.0);

        std::ostringstream os;
        os << "undo_m" << idx << "_i" << iCol;
        glp_set_row_name(lp, row, os.str().c_str());

        rows.push_back(row);
        cols.push_back(iCol);
        coeffs.push_back(-1.0);

        rows.push_back(row);
        cols.push_back(mCol);
        coeffs.push_back(1.0);
      }
    }
  }

}

} // namespace composition::graph
