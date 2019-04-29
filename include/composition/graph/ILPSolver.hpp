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
#include <composition/support/options.hpp>

namespace composition::graph {
using composition::Manifest;
using composition::manifest_idx_t;
using composition::metric::ManifestStats;
using llvm::dbgs;
using composition::support::ILPConnectivityBound;
using composition::support::ILPBlockConnectivityBound;

class ILPSolver {
private:
  int EXPLICIT{};
  int IMPLICIT{};
  int HOTNESS{};
  int HOTNESS_PROTECTEE{};

  glp_prob *lp;
  int cycleCount = 0;
  int connectivityCount = 0;
  int blockConnectivityCount = 0;
  int nOfCount = 0;
  std::function<double(ManifestStats)> costFunction;
  std::vector<int> rows{};
  std::vector<int> cols{};
  std::vector<double> coeffs{};

  boost::bimaps::bimap<int, manifest_idx_t> colsToM{};
  boost::bimaps::bimap<int, manifest_idx_t> colsToE{};
  boost::bimaps::bimap<int, manifest_idx_t> colsToF{};
public:
  ILPSolver();

  virtual ~ILPSolver();

  void destroy();

  void init(int explicitCov, int implicitCov, double hotness, double hotnessProtectee);

  void addManifests(const std::unordered_map<manifest_idx_t, Manifest *> &manifests,
                    std::map<manifest_idx_t, ManifestStats> stats);

  void addDependencies(const std::set<std::pair<manifest_idx_t, manifest_idx_t>> &dependencies);

  void addConflicts(const std::set<std::pair<manifest_idx_t, manifest_idx_t>> &conflicts);

  void addCycles(const std::set<std::set<manifest_idx_t>> &cycles);

  void addConnectivity(const std::set<std::set<manifest_idx_t>> &connectivities);

  void addBlockConnectivity(const std::set<std::set<manifest_idx_t>> &blockConnectivities);

  void setCostFunction(std::function<double(ManifestStats)> f) { this->costFunction = f; }

  std::pair<std::set<manifest_idx_t>, std::set<manifest_idx_t>> run();

  void conflict(std::pair<manifest_idx_t, manifest_idx_t> pair);

  void dependency(std::pair<manifest_idx_t, manifest_idx_t> pair);

  void cycle(const std::set<manifest_idx_t> &ms);

  void edgeConnection(manifest_idx_t edgeInx, std::pair<manifest_idx_t, manifest_idx_t> pair) {
    // e0 depends on m1 and m2; 0 <= m1 + m2 -2 e0 <= 1
    auto row = glp_add_rows(lp, 1);
    glp_set_row_bnds(lp, row, GLP_DB, 0.0, 1.0);
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
    glp_set_row_bnds(lp, row, GLP_DB, 0.0, std::max(size_t(1), edgeDuplicates.size() - 1));
    std::ostringstream os;
    os << "f" << fInx;//<< "_" << edgeDuplicates.size();
    for (auto edgeIndex :edgeDuplicates) {
      os << "_" << edgeIndex;
    }
    glp_set_row_name(lp, row, os.str().c_str());

    rows.push_back(row);
    cols.push_back(colsToF.right.at(fInx));
    coeffs.push_back(
        std::max(size_t(2), edgeDuplicates.size())); // even when there is one edge f need to have a coefficent 2

    llvm::dbgs() << "Duplicates edges covering manifest " << fInx << ":\n";
    for (auto edgeIndex : edgeDuplicates) {
      llvm::dbgs() << edgeIndex << ",";
      rows.push_back(row);
      cols.push_back(colsToE.right.at(edgeIndex));
      coeffs.push_back(-1.0);
    }
    llvm::dbgs() << "\n";
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

    for (auto[fIndex, _, coverage] : implicitCov) {
      rows.push_back(row);
      cols.push_back(colsToF.right.at(fIndex));
      coeffs.push_back(coverage);
    }
  }

  void connectivity(const std::set<manifest_idx_t> &ms, size_t targetConnectivity);

  void blockConnectivity(const std::set<manifest_idx_t> &ms, size_t targetBlockConnectivity);

  void addImplicitCoverage(std::vector<std::tuple<composition::manifest_idx_t,
                                                  std::pair<composition::manifest_idx_t, composition::manifest_idx_t>,
                                                  long unsigned int>> implicitCov,
                           std::map<composition::manifest_idx_t,
                                    std::pair<std::set<composition::manifest_idx_t>,
                                              long unsigned int>> duplicateEdgesOnManifest) {
    // TODO: ensure setting the cost column to zero does not negatively affect the optimization
    for (auto&[eIdx, pair, coverage] : implicitCov) {
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
      rows.push_back(EXPLICIT);
      cols.push_back(col);
      coeffs.push_back(0); // Edges have no explicit coverage

      // implicit
      rows.push_back(IMPLICIT);
      cols.push_back(col);
      coeffs.push_back(0); // Edges have no implicit coverage, f represents such coverages

      // hotness
      rows.push_back(HOTNESS);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no hotness

      // hotnessProtectee
      rows.push_back(HOTNESS_PROTECTEE);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no protectee hotness
    }
    llvm::dbgs() << "Nr. Duplicate edges reported:" << duplicateEdgesOnManifest.size() << "\n";
    // Rows for duplicate edges with f variables
    for (auto&[mIdx, edgecov] : duplicateEdgesOnManifest) {
      auto &[edges, coverage] = edgecov;
      llvm::dbgs() << "f" << mIdx;
      std::ostringstream os;
      os << "f" << mIdx;
      /*for (auto& edge : edges) {
        os << "_" << edge;
        llvm::dbgs() << "_" << edge;
      }
      llvm::dbgs() << "\n";
      */auto col = glp_add_cols(lp, 1);
      glp_set_col_name(lp, col, os.str().c_str()); // assigns name m_n to nth column

      glp_set_col_kind(lp, col, GLP_BV);           // values are binary
      glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0); // values are binary
      glp_set_obj_coef(lp, col, 0);                // TODO: f (edge duplicates) do not impose any costs

      colsToF.insert({col, mIdx});

      // explicit
      rows.push_back(EXPLICIT);
      cols.push_back(col);
      coeffs.push_back(0); // Edges have no explicit coverage

      // implicit
      rows.push_back(IMPLICIT);
      cols.push_back(col);
      /*auto item =
          std::find_if(begin(implicitCov), end(implicitCov), [&mIdx](const auto& e) { return std::get<0>(e) == mIdx; });
      if(item != implicitCov.end()){
	auto &[itemid,manifestmap,coverage] = *item;
        coeffs.push_back(coverage); // TODO: fix implicit value
	llvm::dbgs() << "TEST:"<<mIdx<<"   "<<itemid << "   "<<coverage<<"\n";
      }else
	coeffs.push_back(0);
*/
      coeffs.push_back(coverage);

      // hotness
      rows.push_back(HOTNESS);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no hotness

      // hotnessProtectee
      rows.push_back(HOTNESS_PROTECTEE);
      cols.push_back(col);
      coeffs.push_back(0); // edges have no protectee hotness
    }
    // Add edge constraints, i.e. e = M1 && M2
    for (auto&[eIdx, pair, _] : implicitCov) {
      edgeConnection(eIdx, pair);
    }

    // Add duplicate edge constaints, i.e. f = dup_e1 || dup_e2 || dup_e3
    for (auto&[mIdx, edges] : duplicateEdgesOnManifest) {
      duplicateImplicitEdge(mIdx, edges.first);
    }

    // Add desired implicit coverage
    // implicitCoverageConstraint(implicitCoverageToInstruction, implicitCov, 0.0);
  }

  void addNOfDependencies(std::vector<std::pair<manifest_idx_t,
                                                std::pair<uint64_t, std::vector<manifest_idx_t>>>> nOfs) {
    for (auto[mIdx, nOf] : nOfs) {
      long N = std::min(nOf.first, nOf.second.size());

      // m1 cannot exist without m2 -> m2 - m1 >= 0 - hence m1 depends on m2
      for (auto idx : nOf.second) {
        dependency({idx, mIdx});
      }

      // mX cannot exist without N of m1...mK -> m1 + ... + mK - mX >= (N - 1)
      auto row = glp_add_rows(lp, 1);
      glp_set_row_bnds(lp, row, GLP_LO, N - 1, 0.0);
      std::ostringstream os;
      os << "n_" << N << "_of_" << nOfCount++;
      glp_set_row_name(lp, row, os.str().c_str());

      rows.push_back(row);
      cols.push_back(colsToM.right.at(mIdx));
      coeffs.push_back(-1.0);

      for (auto idx : nOf.second) {
        rows.push_back(row);
        cols.push_back(colsToM.right.at(idx));
        coeffs.push_back(1.0);
      }
    }
  }
}; // namespace composition::graph
} // namespace composition::graph

#endif // COMPOSITION_GRAPH_ILPSOLVER
