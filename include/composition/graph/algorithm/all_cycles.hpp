#ifndef COMPOSITION_GRAPH_ALGORITHM_ALLCYCLES_HPP
#define COMPOSITION_GRAPH_ALGORITHM_ALLCYCLES_HPP

#include <cassert>
#include <deque>
#include <lemon/connectivity.h>
#include <lemon/list_graph.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <map>
#include <set>

namespace composition::graph::algorithm {

class AllCycles {
private:
  std::set<lemon::ListDigraph::Node> blockedSet{};
  std::map<lemon::ListDigraph::Node, std::set<lemon::ListDigraph::Node>> blockedMap{};
  std::deque<lemon::ListDigraph::Node> stack{};

  std::set<std::set<lemon::ListDigraph::Node>> allCycles;

  bool findCyclesInSCG(const lemon::SubDigraph<lemon::ListDigraph> g,
                       const lemon::SubDigraph<lemon::ListDigraph>::Node startVertex,
                       const lemon::SubDigraph<lemon::ListDigraph>::Node currentVertex,
                       const int max) {
    bool foundCycle = false;
    stack.push_front(currentVertex);
    blockedSet.insert(currentVertex);

    for (lemon::SubDigraph<lemon::ListDigraph>::OutArcIt e(g, currentVertex); e != lemon::INVALID; ++e) {
      const auto neighbor = g.target(e);
      if (neighbor == startVertex) {
        // Cycle found
        std::set<lemon::ListDigraph::Node> cycle{};
        for (auto &it : stack) {
          cycle.insert(it);
        }
        size_t pre = allCycles.size();
        allCycles.insert(cycle);
        assert(allCycles.size() == (pre + 1));
        foundCycle = true;
      } else if (blockedSet.find(neighbor) == blockedSet.end()) {
        bool gotCycle = findCyclesInSCG(g, startVertex, neighbor, max);
        foundCycle = foundCycle || gotCycle;
      }

      if (allCycles.size() >= max) {
        return foundCycle;
      }
    }

    if (foundCycle) {
      // remove from blockedSet  and then remove all the other vertices dependent on this vertex from blockedSet
      std::stack<lemon::ListDigraph::Node> queue{};
      queue.push(currentVertex);
      while (!queue.empty()) {
        if (const auto found = blockedMap.find(queue.top()); found != blockedMap.end()) {
          for (const auto node : found->second) {
            queue.push(node);
          }
          blockedMap.erase(found);
        } else {
          blockedSet.erase(queue.top());
          queue.pop();
        }
      }
    } else {
      // if no cycle is found with current vertex then don't unblock it. But find all its neighbors and add this
      // vertex to their blockedMap. If any of those neighbors ever get unblocked then unblock current vertex as well.
      for (lemon::SubDigraph<lemon::ListDigraph>::OutArcIt e(g, currentVertex); e != lemon::INVALID; ++e) {
        auto neighbor = g.target(e);
        blockedMap[neighbor].insert(currentVertex);
      }
    }

    stack.pop_front();
    return foundCycle;
  }

public:
  std::set<std::set<lemon::ListDigraph::Node>> simpleCycles(lemon::ListDigraph &g, int max) {
    lemon::ListDigraph::NodeMap<bool> node_filter{g, true};
    lemon::ListDigraph::ArcMap<bool> arc_filter{g, true};
    lemon::SubDigraph<lemon::ListDigraph> subGraph(g, node_filter, arc_filter);

    for (lemon::ListDigraph::NodeIt n(g); n != lemon::INVALID; ++n) {
      lemon::SubDigraph<lemon::ListDigraph>::NodeMap<int> components{subGraph};
      const int numComponents = lemon::stronglyConnectedComponents(subGraph, components);

      if (numComponents == lemon::countNodes(subGraph)) {
        // each node is a strong component
        // => all cycles are found
        break;
      }

      std::map<int, int> sccs{};
      for (lemon::SubDigraph<lemon::ListDigraph>::NodeIt scNode(subGraph); scNode != lemon::INVALID; ++scNode) {
        sccs[components[scNode]]++;
      }

      int min = numComponents;
      for (lemon::SubDigraph<lemon::ListDigraph>::NodeIt scNode(subGraph); scNode != lemon::INVALID; ++scNode) {
        if (sccs[components[scNode]] == 1) {
          subGraph.disable(scNode);
        } else if (components[scNode] < min) {
          min = components[scNode];
        }
      }

      lemon::ListDigraph::NodeMap<bool> scc_node_filter{g, false};
      for (lemon::SubDigraph<lemon::ListDigraph>::NodeIt scNode(subGraph); scNode != lemon::INVALID; ++scNode) {
        if (components[scNode] == min) {
          scc_node_filter[scNode] = true;
        }
      }
      lemon::SubDigraph<lemon::ListDigraph> sccSubGraph(g, scc_node_filter, arc_filter);
      lemon::SubDigraph<lemon::ListDigraph>::NodeIt start(sccSubGraph);
      findCyclesInSCG(sccSubGraph, start, start, max);
      subGraph.disable(start);

      if (allCycles.size() >= max) {
        return allCycles;
      }
    }

    return allCycles;
  }
};
} // namespace composition::graph::algorithm

#endif // COMPOSITION_GRAPH_ALGORITHM_ALLCYCLES_HPP