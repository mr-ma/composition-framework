#include <catch2/catch.hpp>
#include <composition/graph/algorithm/topological_sort.hpp>

typedef boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS> Graph;

Graph prepare_Graph() {
  Graph G{};

  auto vd0 = boost::add_vertex(G);
  auto vd1 = boost::add_vertex(G);
  auto vd2 = boost::add_vertex(G);
  auto vd3 = boost::add_vertex(G);
  auto vd4 = boost::add_vertex(G);
  auto vd5 = boost::add_vertex(G);

  boost::add_edge(vd0, vd1, G);
  boost::add_edge(vd2, vd4, G);
  boost::add_edge(vd2, vd5, G);
  boost::add_edge(vd0, vd3, G);
  boost::add_edge(vd1, vd4, G);
  boost::add_edge(vd4, vd3, G);

  return G;
}

Graph prepare_multi_Graph() {
  Graph G{};

  auto vd0 = boost::add_vertex(G);
  auto vd1 = boost::add_vertex(G);
  auto vd2 = boost::add_vertex(G);
  auto vd3 = boost::add_vertex(G);
  auto vd4 = boost::add_vertex(G);
  auto vd5 = boost::add_vertex(G);

  boost::add_edge(vd0, vd1, G);
  boost::add_edge(vd0, vd1, G);
  boost::add_edge(vd2, vd4, G);
  boost::add_edge(vd2, vd4, G);
  boost::add_edge(vd2, vd5, G);
  boost::add_edge(vd2, vd5, G);
  boost::add_edge(vd0, vd3, G);
  boost::add_edge(vd0, vd3, G);
  boost::add_edge(vd1, vd4, G);
  boost::add_edge(vd1, vd4, G);
  boost::add_edge(vd4, vd3, G);
  boost::add_edge(vd4, vd3, G);

  return G;
}

TEST_CASE("Topological sort is working", "[topological_sort]") {
  auto G = prepare_Graph();

  std::vector<Graph::vertex_descriptor> c = composition::topological_sort(G);

  std::vector<unsigned long> cInts{};
  auto index = index_map(G);
  for (auto ii = c.rbegin(), ii_end = c.rend(); ii != ii_end; ++ii) {
    cInts.push_back(index[*ii]);
  }

  std::vector<unsigned long> is{2, 5, 0, 1, 4, 3};
  REQUIRE(cInts == is);
}

TEST_CASE("Reverse topological sort is working", "[topological_sort]") {
  auto G = prepare_Graph();

  std::vector<Graph::vertex_descriptor> c = composition::reverse_topological_sort(G);

  std::vector<unsigned long> cInts{};
  auto index = index_map(G);
  for (auto ii = c.rbegin(), ii_end = c.rend(); ii != ii_end; ++ii) {
    cInts.push_back(index[*ii]);
  }

  std::vector<unsigned long> is{3, 4, 1, 0, 5, 2};
  REQUIRE(cInts == is);
}

TEST_CASE("Topological sort is working on multi graphs", "[topological_sort]") {
  auto G = prepare_multi_Graph();

  std::vector<Graph::vertex_descriptor> c = composition::topological_sort(G);

  std::vector<unsigned long> cInts{};
  auto index = index_map(G);
  for (auto ii = c.rbegin(), ii_end = c.rend(); ii != ii_end; ++ii) {
    cInts.push_back(index[*ii]);
  }

  std::vector<unsigned long> is{2, 5, 0, 1, 4, 3};
  REQUIRE(cInts == is);
}

TEST_CASE("Reverse topological sort is working  on multi graphs", "[topological_sort]") {
  auto G = prepare_multi_Graph();

  std::vector<Graph::vertex_descriptor> c = composition::reverse_topological_sort(G);

  std::vector<unsigned long> cInts{};
  auto index = index_map(G);
  for (auto ii = c.rbegin(), ii_end = c.rend(); ii != ii_end; ++ii) {
    cInts.push_back(index[*ii]);
  }

  std::vector<unsigned long> is{3, 4, 1, 0, 5, 2};
  REQUIRE(cInts == is);
}