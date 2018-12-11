#include <composition/metric/Connectivity.hpp>

namespace composition::metric {

Connectivity::Connectivity(std::vector<size_t> v) { updateConnectivity(std::move(v)); }

void Connectivity::updateConnectivity(std::vector<size_t> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  this->avg = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), this->avg)); // NOLINT(modernize-use-transparent-functors)
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  this->variance = sq_sum / static_cast<double>(v.size());
  this->std = std::sqrt(this->variance);
}

void to_json(nlohmann::json& j, const Connectivity& c) {
  j = nlohmann::json{{"avg", c.avg}, {"std", c.std}, {"variance", c.variance}};
}

void from_json(const nlohmann::json& j, Connectivity& c) {
  c.avg = j.at("avg").get<double>();
  c.std = j.at("std").get<double>();
  c.variance = j.at("variance").get<double>();
}
} // namespace composition::metric