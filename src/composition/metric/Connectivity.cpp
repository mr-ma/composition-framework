#include <composition/metric/Connectivity.hpp>

namespace composition {
Connectivity::Connectivity(std::vector<size_t> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  this->avg = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), this->avg));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  this->variance = sq_sum / (double) v.size();
  this->std = std::sqrt(this->variance);
}

Connectivity::Connectivity(double avg, double std) : avg(avg), std(std) {}

Connectivity::Connectivity(std::pair<double, double> c) : Connectivity(c.first, c.second) {}

void to_json(nlohmann::json &j, const Connectivity &c) {
  j = nlohmann::json{
      {"avg", c.avg},
      {"std", c.std},
      {"variance", c.variance}
  };
}

void from_json(const nlohmann::json &j, Connectivity &c) {
  c.avg = j.at("avg").get<double>();
  c.std = j.at("std").get<double>();
  c.variance = j.at("variance").get<double>();
}
}