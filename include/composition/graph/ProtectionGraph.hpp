#ifndef COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
#define COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP

#include <algorithm>
#include <array>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/constraint/constraint.hpp>
#include <composition/graph/constraint/true.hpp>
#include <composition/graph/util/dot.hpp>
#include <composition/graph/util/graphml.hpp>
#include <composition/metric/Performance.hpp>
#include <composition/profiler.hpp>
#include <composition/support/options.hpp>
#include <composition/util/bimap.hpp>
#include <cstdint>
#include <glpk.h>
#include <iterator>
#include <lemon/graph_to_eps.h>
#include <lemon/list_graph.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <random>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace composition::graph {
using composition::graph::constraint::Constraint;
using composition::graph::constraint::constraint_idx_t;
using composition::graph::constraint::True;
using composition::graph::util::graph_to_dot;
using composition::graph::util::graph_to_graphml;
using composition::metric::Performance;
using composition::support::cStats;

using ManifestProtectionMap = composition::util::bimap<manifest_idx_t, manifest_idx_t>;
using ManifestDependencyMap = composition::util::bimap<manifest_idx_t, manifest_idx_t>;

/**
 * The core of the composition framework, the protection graph and its algorithms
 */
class ProtectionGraph {
private:
  /**
   * The graph in boost representation
   */
  lemon::ListDigraph LG{};
  std::unique_ptr<lemon::ListDigraph::NodeMap<vertex_t>> vertices;
  std::unique_ptr<lemon::ListDigraph::ArcMap<edge_t>> edges;
  std::unordered_map<vertex_idx_t, lemon::ListDigraph::Node> VERTICES_DESCRIPTORS{};
  std::unordered_map<edge_idx_t, lemon::ListDigraph::Arc> EDGES_DESCRIPTORS{};

  /**
   * The current strictly increasing vertex index
   */
  vertex_idx_t VertexIdx{};
  /**
   * The current strictly increasing edge index
   */
  edge_idx_t EdgeIdx{};
  /**
   * Cache of vertices for faster lookup
   */
  using vertex_cache_t = std::unordered_map<llvm::Value*, vertex_idx_t>;
  vertex_cache_t vertexRealCache{};
  vertex_cache_t vertexShadowCache{};
  std::array<vertex_cache_t*, 2> vertexCache = {{&vertexRealCache, &vertexShadowCache}};

  /**
   * Map which captures the undo relationship between manifests.
   */
  ManifestDependencyMap DependencyUndo{};
  /**
   * Map which captures the protection relationship between manifests.
   */
  ManifestProtectionMap ManifestProtection{};

  std::unordered_map<manifest_idx_t, Manifest*> MANIFESTS;

  constraint_idx_t ConstraintIdx{};

  boost::bimaps::bimap<boost::bimaps::unordered_set_of<manifest_idx_t>, constraint_idx_t> MANIFESTS_CONSTRAINTS{};
  std::unordered_map<constraint_idx_t, vertex_idx_t> CONSTRAINTS_VERTICES{};
  std::unordered_map<constraint_idx_t, edge_idx_t> CONSTRAINTS_EDGES{};

private:
  /**
   * Adds a vertex with value `v` to the graph if it does not exist.
   * @param v the llvm value
   * @return a vertex descriptor for the added/existing vertex
   */
  vertex_idx_t add_vertex(llvm::Value* value, bool shadow);
  vertex_idx_t add_vertex(
      llvm::Value* value,
      const std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>& constraints);

  /**
   * Adds an edge to the graph.
   * @param s the source vertex
   * @param d the destination vertex
   * @param e the associated edge information
   * @return an edge descriptor pointing to the added edge
   */
  edge_idx_t add_edge(vertex_idx_t s, vertex_idx_t d);
  edge_idx_t add_edge(
      vertex_idx_t s, vertex_idx_t d,
      const std::unordered_map<constraint::constraint_idx_t, std::shared_ptr<constraint::Constraint>>& constraints);

public:
  ProtectionGraph();

  /**
   * Destroys the graph and frees memory
   */
  void destroy();

  size_t countVertices();
  size_t countEdges();

  const ManifestDependencyMap getManifestDependencyMap() const { return DependencyUndo; }

  const ManifestProtectionMap getManifestProtectionMap() const { return ManifestProtection; }

  void addManifests(std::set<Manifest*> manifests);
  void addManifest(Manifest* m);

  void Print(std::string name) {
    auto texts = lemon::ListDigraph::NodeMap<std::string>(LG);
    auto coords = lemon::ListDigraph::NodeMap<lemon::dim2::Point<int>>(LG);
    auto sizes = lemon::ListDigraph::NodeMap<int>(LG);
    int i = 0;
    for (lemon::ListDigraph::NodeIt n(LG); n != lemon::INVALID; ++n) {
      texts[n] = (*vertices)[n].name;
      coords[n] = lemon::dim2::Point<int>(i % 2, i / 2);
      sizes[n] = 1;
      ++i;
    }

    auto widths = lemon::ListDigraph::ArcMap<double>(LG);
    for (lemon::ListDigraph::ArcIt e(LG); e != lemon::INVALID; ++e) {
      widths[e] = 0.5;
    }

    lemon::graphToEps(LG, name + ".eps")
        .nodeSizes(sizes)
        .negateY()
        //.absoluteNodeSizes()
        .coords(coords)
        .nodeTexts(texts)
        .nodeTextSize(0.05)
        .drawArrows()
        .arrowWidth(0.1)
        .edgeWidths(widths)
        .absoluteEdgeWidths()
        .run();
  }

  /**
   * Adds a constraint to the protection graph
   * @param m the manifest associated with the constraint
   * @param c the constraint
   * @return a unique protection index
   */
  constraint_idx_t addConstraint(manifest_idx_t idx, std::shared_ptr<Constraint> c);

  void addHierarchy(llvm::Module& M);

  void connectShadowNodes();

  /**
   * Performs a topological sorting of the given manifests according to the protection graph
   * @param manifests to sort
   * @return the sorted manifests
   */
  std::vector<Manifest*> topologicalSortManifests(std::set<Manifest*> manifests);

  /**
   * Computes the dependency relationship of the manifests.
   */
  void computeManifestDependencies();

  std::set<std::pair<manifest_idx_t, manifest_idx_t>> vertexConflicts();
  std::set<std::pair<manifest_idx_t, manifest_idx_t>> computeDependencies();
  std::set<std::set<manifest_idx_t>> computeCycles();
  std::set<std::set<manifest_idx_t>> computeConnectivity(llvm::Module& M);
  std::set<std::set<manifest_idx_t>> computeBlockConnectivity(llvm::Module& M);

  /**
   * Detects and handles the conflicts in the graph `g`
   * @tparam graph_t the type of the graph `g`
   * @param g the graph
   * @param strategy the strategy to use for handling conflicts
   */
  std::set<Manifest*> randomConflictHandling(llvm::Module& M);
  std::set<Manifest*> ilpConflictHandling(llvm::Module& M, const std::unordered_map<llvm::BasicBlock*, uint64_t>& BFI, size_t totalInstructions);

  template <typename Iter, typename RandomGenerator> Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));
    return start;
  }

  template <typename Iter> Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
  }

  void removeManifest(manifest_idx_t m);
};
} // namespace composition::graph
#endif // COMPOSITION_FRAMEWORK_GRAPH_PROTECTIONGRAPH_HPP
