#ifndef COMPOSITION_GRAPH_ILPSOLVER_HPP
#define COMPOSITION_GRAPH_ILPSOLVER_HPP

#include <boost/bimap/bimap.hpp>
#include <composition/Manifest.hpp>
#include <composition/metric/ManifestStats.hpp>
#include <functional>
#include <glpk.h>
#include <llvm/Support/Debug.h>
#include <map>
#include <set>
#include <vector>

namespace composition::graph {
using composition::Manifest;
using composition::manifest_idx_t;
using composition::metric::ManifestStats;
using llvm::dbgs;

class ILPSolver {
private:
  glp_prob* lp;
  int cycleCount = 0;
  int connectivityCount = 0;
  int blockConnectivityCount = 0;
  std::function<double(ManifestStats)> costFunction;
  std::vector<int> rows{};
  std::vector<int> cols{};
  std::vector<double> coeffs{};

  boost::bimaps::bimap<int, manifest_idx_t> colsToM{};
  boost::bimaps::bimap<int, manifest_idx_t> colsToE{};
  boost::bimaps::bimap<int, manifest_idx_t> colsToF{};

public:
  size_t DesiredConnectivity = 1;
  size_t DesiredBlockConnectivity = 1;

public:
  ILPSolver();

  virtual ~ILPSolver();

  void destroy();

  void init(int explicitCov, int implicitCov, double hotness, double hotnessProtectee);

  void addManifests(std::unordered_map<manifest_idx_t, Manifest*> manifests,
                    std::map<manifest_idx_t, ManifestStats> stats);

  void addDependencies(std::set<std::pair<manifest_idx_t, manifest_idx_t>> dependencies);

  void addConflicts(std::set<std::pair<manifest_idx_t, manifest_idx_t>> conflicts);

  void addCycles(std::set<std::set<manifest_idx_t>> cycles);

  void addConnectivity(std::set<std::set<manifest_idx_t>> connectivities);

  void addBlockConnectivity(std::set<std::set<manifest_idx_t>> blockConnectivities);

  void setCostFunction(std::function<double(ManifestStats)> costFunction) { this->costFunction = costFunction; }

  std::pair<std::set<manifest_idx_t>, std::set<manifest_idx_t>> run();

  void conflict(std::pair<manifest_idx_t, manifest_idx_t> pair);

  void dependency(std::pair<manifest_idx_t, manifest_idx_t> pair);

  void cycle(std::set<manifest_idx_t> ms);

  void edgeConnection(manifest_idx_t edgeInx, std::pair<manifest_idx_t, manifest_idx_t> pair) {
    // e0 depends on m1 and m2; 0 <= m1 + m2 -2 e0 <= 1
    auto row = glp_add_rows(lp, 1);
    glp_set_row_bnds(lp, row, GLP_UP, 0.0, 1.0);
    std::ostringstream os;
    os << "edge_" << edgeInx << "_" << pair.first << "_" << pair.second;
    glp_set_row_name(lp, row, os.str().c_str());

    rows.push_back(row);
    cols.push_back(colsToM.right.at(pair.first));
    coeffs.push_back(1.0);

    rows.push_back(row);
    cols.push_back(colsToM.right.at(pair.second));
    coeffs.push_back(1.0);

    rows.push_back(row);
    cols.push_back(colsToE.right.at(edgeInx));
    coeffs.push_back(-2.0);
  }

  void duplicateImplicitEdge(manifest_idx_t fInx, std::set<manifest_idx_t> edgeDuplicates) {
    // f0 is set if any of the duplicate edges are set (i.e. OR): 0 <= 2f0 - e1 - e2 <=1
    auto row = glp_add_rows(lp, 1);
    glp_set_row_bnds(lp, row, GLP_DB, 0.0, 1.0);
    std::ostringstream os;
    os << "f" << fInx << "_" << edgeDuplicates.size();
    glp_set_row_name(lp, row, os.str().c_str());

    rows.push_back(row);
    cols.push_back(colsToF.right.at(fInx));
    coeffs.push_back(edgeDuplicates.size());

    for (auto edgeIndex : edgeDuplicates) {
      rows.push_back(row);
      cols.push_back(colsToE.right.at(edgeIndex));
      coeffs.push_back(-1.0);
    }
  }

  void implicitCoverageConstraint(
      manifest_idx_t fIndex,
      std::vector<std::tuple<manifest_idx_t /*edge_index*/, std::pair<manifest_idx_t, manifest_idx_t> /*m1 -> m2*/,
                             unsigned long /*coverage*/>>
          implicitCov,
      double targetCoverage) {
    // e1..eN indicate implicit coverage edges
    auto row = glp_add_rows(lp, 1);
    glp_set_row_bnds(lp, row, GLP_LO, targetCoverage, 0.0);
    std::ostringstream os;
    os << "implicit_requirement_" << 0;
    glp_set_row_name(lp, row, os.str().c_str());

    for (auto [fIndex, _, coverage] : implicitCov) {
      rows.push_back(row);
      cols.push_back(colsToF.right.at(fIndex));
      coeffs.push_back(coverage);
    }
  }

  void connectivity(std::set<manifest_idx_t> ms, size_t targetConnectivity);

  void blockConnectivity(std::set<manifest_idx_t> ms, size_t targetBlockConnectivity);

  void addImplicitCoverage(
      std::vector<std::tuple<manifest_idx_t /*edge_index*/, std::pair<manifest_idx_t, manifest_idx_t> /*m1 -> m2*/,
                             unsigned long /*coverage*/>>
          implicitCov,
      std::map<manifest_idx_t /*protected manifest*/, std::set<manifest_idx_t> /*edges*/> duplicateEdgesOnManifest) {
    // TODO: ensure setting the cost column to zero does not negatively affect the optimization
    for (auto& [eIdx, pair, coverage] : implicitCov) {
      llvm::dbgs() << "edge" << eIdx << "_" << pair.first << "_" << pair.second << "\n";
      std::ostringstream os;
      os << "e" << eIdx;
      auto col = glp_add_cols(lp, 1);
      glp_set_col_name(lp, col, os.str().c_str()); // assigns name m_n to nth column

      glp_set_col_kind(lp, col, GLP_BV);           // values are binary
      glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0); // values are binary
      glp_set_obj_coef(lp, col, 0);                // TODO: edges do not impose any costs

      colsToE.insert({col, eIdx});

      // explicit
      rows.push_back(1);
      cols.push_back(col);
      coeffs.push_back(0); // Edges have no explicit coverage

      // implicit
      rows.push_back(2);
      cols.push_back(col);
      coeffs.push_back(coverage);

      // hotness
      rows.push_back(3);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no hotness

      // hotnessProtectee
      rows.push_back(4);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no protectee hotness
    }
    // Rows for duplicate edges with f variables
    for (auto& [mIdx, edges] : duplicateEdgesOnManifest) {
      llvm::dbgs() << "f" << mIdx << "_" << edges.size() << "\n";
      std::ostringstream os;
      os << "f" << mIdx;
      auto col = glp_add_cols(lp, 1);
      glp_set_col_name(lp, col, os.str().c_str()); // assigns name m_n to nth column

      glp_set_col_kind(lp, col, GLP_BV);           // values are binary
      glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0); // values are binary
      glp_set_obj_coef(lp, col, 0);                // TODO: f (edge duplicates) do not impose any costs

      colsToF.insert({col, mIdx});

      // explicit
      rows.push_back(1);
      cols.push_back(col);
      coeffs.push_back(0); // Edges have no explicit coverage

      // implicit
      rows.push_back(2);
      cols.push_back(col);
      coeffs.push_back(277777);

      // hotness
      rows.push_back(3);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no hotness

      // hotnessProtectee
      rows.push_back(4);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no protectee hotness
    }

    for (auto& [mIdx, edges] : duplicateEdgesOnManifest) {
      duplicateImplicitEdge(mIdx, edges);
    }

    // Add implicit coverage edges
    for (auto& [eIdx, pair, _] : implicitCov) {
      edgeConnection(eIdx, pair);
    }

    // Add desired implicit coverage
    // implicitCoverageConstraint(implicitCoverageToInstruction, implicitCov, 0.0);
  }
}; // namespace composition::graph
} // namespace composition::graph

#endif // COMPOSITION_GRAPH_ILPSOLVER