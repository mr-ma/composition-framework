#ifndef COMPOSITION_GRAPH_ILPSOLVER_HPP
#define COMPOSITION_GRAPH_ILPSOLVER_HPP

#include <boost/bimap/bimap.hpp>
#include <composition/Manifest.hpp>
#include <composition/metric/ManifestStats.hpp>
#include <composition/support/options.hpp>
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
using composition::support::ILPBlockConnectivityBound;
using composition::support::ILPConnectivityBound;
using llvm::dbgs;

class ILPSolver {
private:
  int EXPLICIT{};
  int IMPLICIT{};
  int HOTNESS{};
  int HOTNESS_PROTECTEE{};
  int OVERHEAD{};
  int MANIFEST{};
  enum Modes { minOverhead, maxExplicit, maxImplicit, maxConnectivity, maxManifest };
  Modes ObjectiveMode{};
  glp_prob *lp;
  int cycleCount = 0;
  int connectivityCount = 0;
  int blockConnectivityCount = 0;
  int nOfCount = 0;
  int instructionCount = 0;
  int duplicateImplicitEdgeCount = 0;
  std::function<double(ManifestStats)> costFunction;
  std::vector<int> rows{};
  std::vector<int> cols{};
  std::vector<double> coeffs{};

  boost::bimaps::bimap<int, manifest_idx_t> colsToM{};
  boost::bimaps::bimap<int, manifest_idx_t> colsToE{};
  boost::bimaps::bimap<int, manifest_idx_t> colsToF{};
  std::unordered_map<llvm::Instruction *, int> ItoCols{};
  const std::string MANIFEST_OBJ = "manifest";
  const std::string OVERHEAD_OBJ = "overhead";
  const std::string EXPLICIT_OBJ = "explicit";
  const std::string IMPLICIT_OBJ = "implicit";
  const std::string CONNECTIVITY_OBJ = "connectivity";

public:
  ILPSolver();

  virtual ~ILPSolver();

  void destroy();

  void init(const std::string &objective, double overheadBound, int explicitCov, int implicitCov, double hotness,
            double hotnessProtectee);

  void addManifests(const std::unordered_map<manifest_idx_t, Manifest *> &manifests,
                    std::map<manifest_idx_t, ManifestStats> stats);

  void addDependencies(const std::set<std::pair<manifest_idx_t, manifest_idx_t>> &dependencies);

  void addConflicts(const std::set<std::pair<manifest_idx_t, manifest_idx_t>> &conflicts);

  void addCycles(const std::set<std::set<manifest_idx_t>> &cycles);

  void addConnectivity(const std::set<std::set<manifest_idx_t>> &connectivities);

  void addBlockConnectivity(const std::set<std::set<manifest_idx_t>> &blockConnectivities);

  void addExplicitCoverages(const std::map<llvm::Instruction *, std::set<manifest_idx_t>> &coverage);

  void addUndoDependencies(const std::unordered_map<manifest_idx_t, Manifest *> &manifests);

  void setCostFunction(std::function<double(ManifestStats)> f) { this->costFunction = f; }

  std::pair<std::set<manifest_idx_t>, std::set<manifest_idx_t>> run();

  void conflict(std::pair<manifest_idx_t, manifest_idx_t> pair);

  void dependency(std::pair<manifest_idx_t, manifest_idx_t> pair);

  void cycle(const std::set<manifest_idx_t> &ms);

  void setMode(const std::string &obj) {
    if (obj == OVERHEAD_OBJ) {
      ObjectiveMode = minOverhead;
    } else if (obj == EXPLICIT_OBJ) {
      ObjectiveMode = maxExplicit;
    } else if (obj == IMPLICIT_OBJ) {
      ObjectiveMode = maxImplicit;
    } else if (obj == CONNECTIVITY_OBJ) {
      ObjectiveMode = maxConnectivity;
    } else if (obj == MANIFEST_OBJ) {
      ObjectiveMode = maxManifest;
    } else {
      // default is minOverhead
      ObjectiveMode = minOverhead;
    }
  }

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

  void duplicateImplicitEdge(manifest_idx_t fInx, const std::set<manifest_idx_t> &edgeDuplicates) {
    // f0 is set if any of the duplicate edges are set (i.e. OR): 0 <= 2f0 - e1 - e2 <=1
    auto row = glp_add_rows(lp, 1);
    glp_set_row_bnds(lp, row, GLP_DB, 0.0, std::max(size_t(1), edgeDuplicates.size() - 1));
    std::ostringstream os;
    os << "f" << fInx; //<< "_" << edgeDuplicates.size();
    /*for (auto edgeIndex : edgeDuplicates) {
      os << "_" << edgeIndex;
    }*/
    os << "_" << duplicateImplicitEdgeCount;
    glp_set_row_name(lp, row, os.str().c_str());

    rows.push_back(row);
    cols.push_back(colsToF.right.at(fInx));
    coeffs.push_back(
        std::max(size_t(2), edgeDuplicates.size())); // even when there is one edge f need to have a coefficent 2

    //llvm::dbgs() << "Duplicates edges covering manifest " << fInx << ":\n";
    for (auto edgeIndex : edgeDuplicates) {
      //llvm::dbgs() << edgeIndex << ",";
      rows.push_back(row);
      cols.push_back(colsToE.right.at(edgeIndex));
      coeffs.push_back(-1.0);
    }
    //llvm::dbgs() << "\n";
  }

  void connectivity(const std::set<manifest_idx_t> &ms, double targetConnectivity);

  void blockConnectivity(const std::set<manifest_idx_t> &ms, double targetBlockConnectivity);

  void explicitCoverage(llvm::Instruction *I, const std::set<manifest_idx_t> &ms);

  double get_obj_coef_manifest(double overheadValue) {
    //this is only called for manifests and thus no implicit/explicit value is needed,
    //implicit/explicit values are only on edges not manifests!
    switch (ObjectiveMode) {
    case minOverhead:return overheadValue;
    case maxManifest:return 1; //every manifest has weight of 1
    default:return 0;
      // TODO: case maxConnectivity:
      // return connectivity is not a column yet!
    }
  }
  double get_obj_coef_edge(long unsigned int coverage) {
    switch (ObjectiveMode) {
    case maxImplicit:return coverage;
    default:return 0;
    }
  }

  double get_obj_coef_explicit(long unsigned int coverage) {
    switch (ObjectiveMode) {
    case maxExplicit:return coverage;
    default:return 0;
    }
  }

  int get_obj_dir() {
    switch (ObjectiveMode) {
    case minOverhead:return GLP_MIN;
    default:return GLP_MAX;
    }
  }
  void printModeILPResults() {
    std::string objective;
    uint64_t explicit_re = 0;
    uint64_t implicit_re = 0;
    double overhead_re = 0.0;
    double connectivity = 0.0;

    switch (ObjectiveMode) {
    case minOverhead:objective = "min Overhead. ";
      explicit_re = (uint64_t) glp_mip_row_val(lp, EXPLICIT);
      implicit_re = (uint64_t) glp_mip_row_val(lp, IMPLICIT);
      overhead_re = (double) glp_mip_obj_val(lp);
      break;
    case maxExplicit:objective = "max Explicit. ";
      explicit_re = (uint64_t) glp_mip_obj_val(lp);
      implicit_re = (uint64_t) glp_mip_row_val(lp, IMPLICIT);
      overhead_re = (double) glp_mip_row_val(lp, OVERHEAD);
      break;
    case maxImplicit:objective = "max Implicit. ";
      explicit_re = (uint64_t) glp_mip_row_val(lp, EXPLICIT);
      overhead_re = (double) glp_mip_row_val(lp, OVERHEAD);
      implicit_re = (uint64_t) glp_mip_obj_val(lp);
      break;
    case maxConnectivity:objective = "max Connectivity. ";
      // TODO: print for connectivity
      break;
    case maxManifest:objective = "max manifest. ";
      break;
    default:break;
    }

    llvm::dbgs() << "ILP resuls. " << objective << " overhead: " << overhead_re << " explicit instruction coverage: "
                 << explicit_re << " implicit instruction coverage: " << implicit_re << "\n";
  }
  void addModeRows(double overheadBound,
                   int explicitBound,
                   int implicitBound,
                   double hotness,
                   double hotnessProtectee) {
    switch (ObjectiveMode) {
    case minOverhead:
      // row 1
      EXPLICIT = glp_add_rows(lp, 1);
      glp_set_row_name(lp, EXPLICIT, "explicit");                 // assigns name p to first row
      glp_set_row_bnds(lp, EXPLICIT, GLP_LO, explicitBound, 0.0); // 0 < explicit <= inf
      // row 2
      IMPLICIT = glp_add_rows(lp, 1);
      glp_set_row_name(lp, IMPLICIT, "implicit");                 // assigns name q to second row
      glp_set_row_bnds(lp, IMPLICIT, GLP_LO, implicitBound, 0.0); // 0 < implicit <= inf

      MANIFEST = glp_add_rows(lp, 1);
      glp_set_row_name(lp, MANIFEST, "manifest");
      glp_set_row_bnds(lp, MANIFEST, GLP_LO, 0.0, 0.0);
      break;
    case maxExplicit:
      // row 1
      IMPLICIT = glp_add_rows(lp, 1);
      glp_set_row_name(lp, IMPLICIT, "implicit");                 // assigns name q to second row
      glp_set_row_bnds(lp, IMPLICIT, GLP_LO, implicitBound, 0.0); // 0 < implicit <= inf
      //row 2
      OVERHEAD = glp_add_rows(lp, 1);
      glp_set_row_name(lp, OVERHEAD, "overhead");                 // assigns name p to first row
      if (overheadBound > 0) {
        glp_set_row_bnds(lp, OVERHEAD, GLP_UP, 0.0, overheadBound); // 0 < overhead <= inf
      } else {
        glp_set_row_bnds(lp, OVERHEAD, GLP_LO, overheadBound, 0); // 0 < overhead <= inf
      }
      MANIFEST = glp_add_rows(lp, 1);
      glp_set_row_name(lp, MANIFEST, "manifest");
      glp_set_row_bnds(lp, MANIFEST, GLP_LO, 0.0, 0.0);
      break;
    case maxImplicit:
      // row 1
      EXPLICIT = glp_add_rows(lp, 1);
      glp_set_row_name(lp, EXPLICIT, "explicit");                 // assigns name p to first row
      glp_set_row_bnds(lp, EXPLICIT, GLP_LO, explicitBound, 0.0); // 0 < explicit <= inf
      // row 2
      OVERHEAD = glp_add_rows(lp, 1);
      glp_set_row_name(lp, OVERHEAD, "overhead");                 // assigns name p to first row
      if (overheadBound > 0) {
        glp_set_row_bnds(lp, OVERHEAD, GLP_UP, 0.0, overheadBound); // 0 < overhead <= inf
      } else {
        glp_set_row_bnds(lp, OVERHEAD, GLP_LO, overheadBound, 0); // 0 < overhead <= inf
      }
      MANIFEST = glp_add_rows(lp, 1);
      glp_set_row_name(lp, MANIFEST, "manifest");
      glp_set_row_bnds(lp, MANIFEST, GLP_LO, 0.0, 0.0);
      break;
    case maxManifest:
      // row 1
      EXPLICIT = glp_add_rows(lp, 1);
      glp_set_row_name(lp, EXPLICIT, "explicit");                 // assigns name p to first row
      glp_set_row_bnds(lp, EXPLICIT, GLP_LO, explicitBound, 0.0); // 0 < explicit <= inf
      // row 2
      IMPLICIT = glp_add_rows(lp, 1);
      glp_set_row_name(lp, IMPLICIT, "implicit");                 // assigns name q to second row
      glp_set_row_bnds(lp, IMPLICIT, GLP_LO, implicitBound, 0.0); // 0 < implicit <= inf
      // row 3
      OVERHEAD = glp_add_rows(lp, 1);
      glp_set_row_name(lp, OVERHEAD, "overhead");                 // assigns name p to first row
      if (overheadBound > 0) {
        glp_set_row_bnds(lp, OVERHEAD, GLP_UP, 0.0, overheadBound); // 0 < overhead <= inf
      } else {
        glp_set_row_bnds(lp, OVERHEAD, GLP_LO, overheadBound, 0); // 0 < overhead <= inf
      }
      break;

    case maxConnectivity:
      // TODO: maximize connectivity?
    default:break;
    }
    // row 3
    HOTNESS = glp_add_rows(lp, 1);
    glp_set_row_name(lp, HOTNESS, "hotness");            // assigns name q to second row
    glp_set_row_bnds(lp, HOTNESS, GLP_LO, hotness, 0.0); // 0 < unique <= inf
    // row 4
    HOTNESS_PROTECTEE = glp_add_rows(lp, 1);
    glp_set_row_name(lp, HOTNESS_PROTECTEE, "hotnessProtectee");            // assigns name q to second row
    glp_set_row_bnds(lp, HOTNESS_PROTECTEE, GLP_LO, hotnessProtectee, 0.0); // 0 < unique <= inf
  }
  void addModeColumns(const int col, const double overheadValue, const int explicitValue, const int implicitValue,
                      const double hotnessValue, const double blockHotnessValue, const int manifestValue) {
    switch (ObjectiveMode) {
    case minOverhead:
      // explicit
      rows.push_back(EXPLICIT);
      cols.push_back(col);
      coeffs.push_back(explicitValue);

      rows.push_back(IMPLICIT);
      cols.push_back(col);
      coeffs.push_back(implicitValue);

      rows.push_back(MANIFEST);
      cols.push_back(col);
      coeffs.push_back(manifestValue);

      break;
    case maxExplicit:
      // implicit
      rows.push_back(IMPLICIT);
      cols.push_back(col);
      coeffs.push_back(implicitValue);
      // overhead
      rows.push_back(OVERHEAD);
      cols.push_back(col);
      coeffs.push_back(overheadValue);

      rows.push_back(MANIFEST);
      cols.push_back(col);
      coeffs.push_back(manifestValue);
      break;
    case maxImplicit:
      // explicit
      rows.push_back(EXPLICIT);
      cols.push_back(col);
      coeffs.push_back(explicitValue);
      // overhead
      rows.push_back(OVERHEAD);
      cols.push_back(col);
      coeffs.push_back(overheadValue);

      rows.push_back(MANIFEST);
      cols.push_back(col);
      coeffs.push_back(manifestValue);
      break;
    case maxManifest:
      // explicit
      rows.push_back(EXPLICIT);
      cols.push_back(col);
      coeffs.push_back(explicitValue);

      // manifest has no implicit coverage but edges do
      rows.push_back(IMPLICIT);
      cols.push_back(col);
      coeffs.push_back(implicitValue);
      // overhead
      rows.push_back(OVERHEAD);
      cols.push_back(col);
      coeffs.push_back(overheadValue);
      break;

    case maxConnectivity:
      // TODO: maximize connectivity?
    default:break;
    }
    // hotness
    rows.push_back(HOTNESS);
    cols.push_back(col);
    coeffs.push_back(hotnessValue);

    // hotnessProtectee
    rows.push_back(HOTNESS_PROTECTEE);
    cols.push_back(col);
    coeffs.push_back(blockHotnessValue);
  }
  /*
  void addImplicitCoverage(
      const std::vector<std::tuple<composition::manifest_idx_t,
                                   std::pair<composition::manifest_idx_t, composition::manifest_idx_t>,
                                   long unsigned int>> &
      implicitCov,
      const std::map<composition::manifest_idx_t, std::pair<std::set<composition::manifest_idx_t>, long unsigned int>> &
      duplicateEdgesOnManifest) {
    // TODO: ensure setting the cost column to zero does not negatively affect the optimization
    for (auto&[eIdx, pair, coverage] : implicitCov) {
      //llvm::dbgs() << "edge" << eIdx << "_" << pair.first << "_" << pair.second << "\n";
      std::ostringstream os;
      os << "e" << eIdx;
      auto col = glp_add_cols(lp, 1);
      glp_set_col_name(lp, col, os.str().c_str()); // assigns name m_n to nth column

      glp_set_col_kind(lp, col, GLP_BV);           // values are binary
      glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0); // values are binary
      glp_set_obj_coef(lp, col, 0);                // TODO: edges do not impose any costs

      colsToE.insert({col, eIdx});

      addModeColumns(col, 0, 0, 0, 0, 0, 0);
    }
    //llvm::dbgs() << "Nr. Duplicate edges reported:" << duplicateEdgesOnManifest.size() << "\n";
    // Rows for duplicate edges with f variables
    for (auto&[mIdx, edgecov] : duplicateEdgesOnManifest) {
      auto&[edges, coverage] = edgecov;
      //llvm::dbgs() << "f" << mIdx;
      std::ostringstream os;
      os << "f" << mIdx;
      auto col = glp_add_cols(lp, 1);
      glp_set_col_name(lp, col, os.str().c_str()); // assigns name m_n to nth column

      glp_set_col_kind(lp, col, GLP_BV);                      // values are binary
      glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0);            // values are binary
      glp_set_obj_coef(lp, col, get_obj_coef_edge(coverage)); // TODO: f (edge duplicates) do not impose any costs

      colsToF.insert({col, mIdx});
      addModeColumns(col, 0, 0, coverage, 0, 0, 0);
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
  }*/

  void addNewImplicitCoverage() {
    // Rules
    /**
     * (1) For each instruction (i_i) that is explicitly covered, introduce a copied implicit instruction variable (implicit_i)
     *
     * (2) The implicit instruction variable is binary
     *
     * (3) If the instruction is not explicitly covered, it cannot be implicitly covered
     *
     * (4) Instruction are implictly covered if we have a manifest protecting a manifests explicit covered instruction
     */

    // Idea
    /**
     * (4 constraint) implicit_i = 1 <=> i_i and (any manifest protecting i_i) and (any manifest protecting a manifest that explicitly protects i_i)
     *
     * By definition we know that i_i is protected iff a corresponding manifest protects it, i.e., we can simplify to:
     *
     * (4 simplified) implicit_i = 1 <=> i_i and (any manifest protecting a manifest that explicitly protects i_i)
     *
     * We can substiute/define (any manifest protecting a manifest that explicitly protects i_i) as:
     * * (4.1) mj protects i_i, mk protects mj
     * * (4.2) f_mj_mk = mj + mk -1
     * * (4.3) implicit_i = 1 <=> i_i and (any f_mj_mk)
     */

    // Constraints
    /**
     * (1-3) i_i >= implicit_i
     * (4) implicit_i = 1 <=> any(f_mj_mk)
     * (4 implicit_i_row bounds) 0 <= implicit_i <= std::max(1, count(f_mj_mk) - 1)
     * (4 constraint) N = std::max(2, count(f_mj_mk));  -f_mj_mk + N * implicit_i - implicit_i_row = 0
     *
     * Examples:
     * One manifest: -f_mj_mk + 2 * implicit_i - implicit_i_row = 0; 0 <= implicit_i_row <= 1
     * Two manifests: 2 * -f_mj_mk + 2 * implicit_i - implicit_i_row = 0; 0 <= implicit_i_row <= 1
     * Three manifests: 3 * -f_mj_mk + 3 * implicit_i - implicit_i_row = 0; 0 <= implicit_i_row <= 2
     *
     */

  }

  void
  addNOfDependencies(const std::vector<std::pair<manifest_idx_t,
                                                 std::pair<uint64_t, std::vector<manifest_idx_t>>>> &nOfs) {
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
