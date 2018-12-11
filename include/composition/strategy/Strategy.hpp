#ifndef COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP
#define COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP

#include <composition/Manifest.hpp>

namespace composition::strategy {
/**
 * Abstract class to define a structure for strategies
 */
class Strategy {
public:
  virtual ~Strategy() = default;

  /**
   * Algorithm to find a manifest that should be removed upon finding a cycle in the graph.
   * @param manifests the list of manifests which are in conflict
   * @return the manifest to remove
   */
  virtual Manifest* decideCycle(std::vector<Manifest*> manifests) = 0;

  /**
   * Algorithm to find a manifest that should be removed upon finding a present/preserved conflict in the graph.
   * @param manifests the list of manifests which are in conflict
   * @return the manifest to remove
   */
  virtual Manifest* decidePresentPreserved(std::vector<Manifest*> manifests) = 0;
};
} // namespace composition::strategy

#endif // COMPOSITION_FRAMEWORK_STRATEGY_STRATEGY_HPP
