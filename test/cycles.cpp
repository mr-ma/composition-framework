#include <catch2/catch.hpp>
#include <composition/graph/algorithm/all_cycles.hpp>
#include <lemon/list_graph.h>

TEST_CASE("Repeated SCC detects all cycles", "[lemon]") {
  lemon::ListDigraph g{};
  auto n1 = g.addNode();
  auto n2 = g.addNode();
  auto n3 = g.addNode();
  auto n4 = g.addNode();
  auto n5 = g.addNode();
  auto n6 = g.addNode();
  auto n7 = g.addNode();
  auto n8 = g.addNode();
  auto n9 = g.addNode();

  auto e12 = g.addArc(n1, n2);
  auto e18 = g.addArc(n1, n8);

  auto e23 = g.addArc(n2, n3);
  auto e27 = g.addArc(n2, n7);
  auto e29 = g.addArc(n2, n9);

  auto e31 = g.addArc(n3, n1);
  auto e34 = g.addArc(n3, n4);
  auto e36 = g.addArc(n3, n6);

  auto e45 = g.addArc(n4, n5);

  auto e52 = g.addArc(n5, n2);

  auto e64 = g.addArc(n6, n4);

  auto e89 = g.addArc(n8, n9);
  auto e98 = g.addArc(n9, n8);

  composition::graph::algorithm::AllCycles a{};
  auto all = a.simpleCycles(g);
  REQUIRE(all.size() == 4);
  for (auto cycle : all) {
    WARN("Cycle: ");
    for (auto n : cycle) {
      WARN((g.id(n) + 1) << " -> ");
    }
    WARN("\n");
  }
}