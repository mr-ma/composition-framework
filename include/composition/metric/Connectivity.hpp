#ifndef COMPOSITION_FRAMEWORK_METRIC_CONNECTIVITY_HPP
#define COMPOSITION_FRAMEWORK_METRIC_CONNECTIVITY_HPP
#include <utility>
#include <cstddef>
#include <nlohmann/json.hpp>

namespace composition::metric { 
/**
 * Helper class to calculate the connectivity of something
 */
struct Connectivity {
  double avg{};
  double std{};
  double variance{};

  Connectivity() = default;

  /**
   * Constructor which accepts a list of element of type size_t to calculate the connectivity.
   * @param v a vector of measurements
   */
  explicit Connectivity(std::vector<size_t> v);

  /**
   * Calculates and updates the connectivity.
   * @param v a vector of measurements
   */
  void updateConnectivity(std::vector<size_t> v);
};

/**
 * Converts `Connectivity` into JSON representation.
 * @param j IN/OUT the resulting JSON
 * @param c IN the connectivity
 */
void to_json(nlohmann::json &j, const Connectivity &c);

/**
 * Converts JSON into `Connectivity` representation
 * @param j IN the JSON source
 * @param c IN/OUT the resulting connectivity
 */
void from_json(const nlohmann::json &j, Connectivity &c);
}
#endif //COMPOSITION_FRAMEWORK_METRIC_CONNECTIVITY_HPP
