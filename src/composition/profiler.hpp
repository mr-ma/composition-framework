#ifndef COMPOSITION_FRAMEWORK_PROFILER_HPP
#define COMPOSITION_FRAMEWORK_PROFILER_HPP

#include <utility>
#include <string>
#include <chrono>

class Profiler {
  std::chrono::high_resolution_clock::time_point p;

public:
  explicit Profiler() {
    reset();
  }

  void reset() {
    p = std::chrono::high_resolution_clock::now();
  }

  double stop() {
    auto d = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - p);
    return d.count();
  }
};

#endif //COMPOSITION_FRAMEWORK_PROFILER_HPP
