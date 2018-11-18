#ifndef COMPOSITION_FRAMEWORK_PROFILER_HPP
#define COMPOSITION_FRAMEWORK_PROFILER_HPP

#include <utility>
#include <string>
#include <chrono>

/**
 * Simple profiler with high resolution which measures the result in seconds.
 */
class Profiler {
  std::chrono::high_resolution_clock::time_point t1;

public:
  explicit Profiler() {
    reset();
  }

  /**
   * Restart the profiler
   */
  void reset() {
    t1 = std::chrono::high_resolution_clock::now();
  }

  /**
   * Stop the profiler and measure elapsed time in seconds
   * @return the time in seconds
   */
  double stop() {
    auto t2 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  }
};

#endif //COMPOSITION_FRAMEWORK_PROFILER_HPP
