#ifndef COMPOSITION_FRAMEWORK_METRIC_CONNECTIVITY_HPP
#define COMPOSITION_FRAMEWORK_METRIC_CONNECTIVITY_HPP
#include <utility>
#include <cstddef>
#include <nlohmann/json.hpp>

namespace composition {
struct Connectivity {
  double avg{};
  double std{};

  Connectivity() = default;

  Connectivity(double avg, double std);

  explicit Connectivity(std::pair<double, double> c);

  explicit Connectivity(std::vector<size_t> v);
};

void to_json(nlohmann::json &j, const Connectivity &c);

void from_json(const nlohmann::json &j, Connectivity &c);
}
#endif //COMPOSITION_FRAMEWORK_METRIC_CONNECTIVITY_HPP
