#ifndef LEPP3_DEBUG_TIMER_H__
#define LEPP3_DEBUG_TIMER_H__

#include <cstdlib>
#include <sys/time.h>

namespace lepp {

/**
 * The class allows us to measure (in a very simplistic way) the time
 * elapsed between the start and stop of the timer, giving a way to
 * perform some basic benchmarking.
 */
class Timer {
public:
  /**
   * Starts the timer.
   */
  timeval start() {
    gettimeofday(&this->timer[0], NULL);
    return this->timer[0];
  }

  /**
   * Stops the timer.
   * Performs no check on whether the timer is already started.
   */
  timeval stop() {
    gettimeofday(&this->timer[1], NULL);
    return this->timer[1];
  }

  /**
   * Gets the duration between the start and stop calls in miliseconds.
   */
  int duration() const {
    int secs = this->timer[1].tv_sec - this->timer[0].tv_sec;
    int usecs = this->timer[1].tv_usec - this->timer[0].tv_usec;

    if (usecs < 0) {
      --secs;
      usecs += 1000000;
    }

    return static_cast<int>(secs * 1000 + usecs / 1000.0 + 0.5);
  }
private:
  timeval timer[2];
};

}  // namespace lepp
#endif
