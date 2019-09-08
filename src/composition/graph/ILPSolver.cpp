#include <composition/graph/ILPSolver.hpp>
#include <composition/support/options.hpp>

namespace composition::graph {

ILPSolver::ILPSolver() {
}

ILPSolver::~ILPSolver() { destroy(); }

void ILPSolver::destroy() {
  lp.Reset();
}

void ILPSolver::init(const std::string &objectiveMode, double overheadBound, int explicitBound, int implicitBound,
                     double hotness, double hotnessProtectee) {
  this->setMode(objectiveMode);
  //after setting the mode we can set the objective direction, min or max
  objective = lp.MutableObjective();
  addModeRows(overheadBound, explicitBound, implicitBound, hotness, hotnessProtectee);
}

void ILPSolver::addManifests(const std::unordered_map<manifest_idx_t, Manifest *> &manifests,
                             std::map<manifest_idx_t, ManifestStats> stats) {
  // COLUMNS
  for (auto&[mIdx, m] : manifests) {
    // column N
    std::ostringstream os;
    os << "m" << m->index;
    auto col = lp.MakeBoolVar(os.str());
    objective->SetCoefficient(col, get_obj_coef_manifest(costFunction(stats[mIdx]))); // costs

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
  switch (ObjectiveMode) {
    case minOverhead:
      objective->SetMinimization();
    break;
    default:
      objective->SetMaximization();
    break;
  }

  llvm::dbgs() << "Number of variables = " << lp.NumVariables() << "\n";
  llvm::dbgs() << "Number of constraints = " << lp.NumConstraints() << "\n";

  lp.SetNumThreads(8);
  lp.EnableOutput();
  
  lp.MakeSum({});

  const MPSolver::ResultStatus result_status = lp.Solve();
  // Check that the problem has an optimal solution.
  if (result_status != MPSolver::OPTIMAL) {
    llvm::dbgs() << "The problem does not have an optimal solution!\n";
    assert(false);
  }
  
  if (!composition::support::ILPProblem.empty()) {
    // Write problem definition (Gurobi)
    llvm::dbgs() << "Writing problem to" << composition::support::ILPProblem.getValue() << "\n";
    lp.Write(composition::support::ILPProblem.getValue());
  }

  auto objective_result = objective->Value();

  std::set<manifest_idx_t> acceptedManifests{};
  for (auto&[col, mIdx] : colsToM) {
    if (col->solution_value() == 1) {
      acceptedManifests.insert(mIdx);
      // TODO: calculate implicit coverage based on the accepted edges
    }
  }

  std::set<manifest_idx_t> acceptedEdges{};
  for (auto&[col, eIdx] : colsToE) {
    if (col->solution_value() == 1) {
      acceptedEdges.insert(eIdx);
    }
  }
  printModeILPResults();

  return {acceptedManifests, acceptedEdges};
}

void ILPSolver::conflict(std::pair<manifest_idx_t, manifest_idx_t> pair) {
  // m1 and m2 conflict; m1 + m2 <= 1
  std::ostringstream os;
  os << "conflict_" << pair.first << "_" << pair.second;

  auto row = lp.MakeRowConstraint(-lp.infinity(), 1.0, os.str());
  row->SetCoefficient(colsToM.right.at(pair.first), 1.0);
  row->SetCoefficient(colsToM.right.at(pair.second), 1.0);
}

void ILPSolver::dependency(std::pair<manifest_idx_t, manifest_idx_t> pair) {
  // m1 depends on m2; m1 <= m2; m1 - m2 <= 0
  std::ostringstream os;
  os << "dependency_" << pair.first << "_" << pair.second;

  auto row = lp.MakeRowConstraint(-lp.infinity(), 0.0, os.str());
  row->SetCoefficient(colsToM.right.at(pair.first), 1.0);
  row->SetCoefficient(colsToM.right.at(pair.second), -1.0);
}

void ILPSolver::cycle(const std::set<manifest_idx_t> &ms) {
  // m1..mN form a cycle; m1+m2+..+mN <= N-1
  std::ostringstream os;
  os << "cycle_" << cycleCount++;

  auto row = lp.MakeRowConstraint(-lp.infinity(), ms.size() - 1, os.str());
  for (auto &idx : ms) {
    row->SetCoefficient(colsToM.right.at(idx), 1.0);
  }
}

void ILPSolver::connectivity(const std::set<manifest_idx_t> &ms, double targetConnectivity) {
  // m1..mN protect an Instruction; m1+m2+..+mN >= min(N, targetConnectivity)
  std::ostringstream os;
  os << "connectivity_" << connectivityCount++;

  auto row = lp.MakeRowConstraint(std::min((double) ms.size(), targetConnectivity), lp.infinity(), os.str());

  for (auto &idx : ms) {
    row->SetCoefficient(colsToM.right.at(idx), 1.0);
  }
}

void ILPSolver::blockConnectivity(const std::set<manifest_idx_t> &ms, double targetBlockConnectivity) {
  // m1..mN protect a BasicBlock; m1+m2+..+mN >= min(N, targetBlockConnectivity)
  std::ostringstream os;
  os << "block_connectivity_" << blockConnectivityCount++;

  auto row = lp.MakeRowConstraint(std::min((double) ms.size(), targetBlockConnectivity), lp.infinity(), os.str());

  for (auto &idx : ms) {
    row->SetCoefficient(colsToM.right.at(idx), 1.0);
  }
}

void ILPSolver::explicitCoverage(llvm::Instruction *I, const std::set<manifest_idx_t> &ms) {
  std::ostringstream os;
  os << "i" << instructionCount;

  auto col = lp.MakeBoolVar(os.str());
  objective->SetCoefficient(col, get_obj_coef_explicit(1));

  ItoCols.insert({I, col});
  addModeColumns(col, 0, 1 /*explicit cov of instruction*/, 0, 0, 0, 0);

  // any of m1...mN if c
  os << "_row";
  auto orRow = lp.MakeRowConstraint(0.0, std::max(size_t(1), ms.size() - 1), os.str());
  orRow->SetCoefficient(col, std::max(size_t(2), ms.size()));

  for (auto m : ms) {
    orRow->SetCoefficient(colsToM.right.at(m), -1.0);
  }
  instructionCount++;
}

void ILPSolver::addUndoDependencies(const std::unordered_map<manifest_idx_t, Manifest *> &manifests) {
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
        std::ostringstream os;
        os << "undo_m" << idx << "_" << iCol->name();
        
        auto row = lp.MakeRowConstraint(0.0, 0.0, os.str());
        row->SetCoefficient(iCol, -1.0);
        row->SetCoefficient(mCol, 1.0);
      }
    }
  }

}

} // namespace composition::graph
