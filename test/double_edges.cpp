#include <catch2/catch.hpp>
#include <lemon/list_graph.h>

TEST_CASE("Adding edges twice results in a single edge", "[lemon]") {
  lemon::ListDigraph g{};
  auto x = g.addNode();
  auto y = g.addNode();

  auto e1 = g.addArc(x, y);

  REQUIRE(lemon::countNodes(g) == 2);
  REQUIRE(lemon::countArcs(g) == 1);

  auto e2 = g.addArc(y, x);
  REQUIRE(lemon::countArcs(g) == 2);
}