#ifndef LEPP3_HIRESTIMER_H_
#define LEPP3_HIRESTIMER_H_

#include <chrono>

namespace lepp {

/**
 * High-resolution timer for measuring the elapsed time between start and stop calls.
 * Useful for timing performance.
 */
class HiResTimer {
public:

  // Use the highest resolution clock available
  typedef std::chrono::high_resolution_clock clock;

  /**
   * Starts the timer.
   */
  clock::time_point start() {
    startTime_ = clock::now();
    running_ = true;
    return startTime_;
  }

  /**
   * Stops the timer.
   * Performs no check on whether the timer is already started.
   */
  clock::time_point stop() {
    endTime_ = clock::now();
    running_ = false;
    return endTime_;
  }

  /**
   * Gets the duration between the start and stop calls in milliseconds.
   */
  double duration() const {
    return std::chrono::duration<double, std::milli>(endTime_ - startTime_).count();
  }

  /**
   * Gets the current state of the timer.
   * Returns TRUE if start has been called and stop has not been called.
   * Returns FALSE otherwise.
   */
  bool running() const {
      return running_;
  }

private:
  clock::time_point startTime_;
  clock::time_point endTime_;
  bool running_ = false;
};

}  // namespace lepp

#endif // LEPP3_HIRESTIMER_H_
