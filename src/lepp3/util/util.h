#ifndef LEPP3_UTIL_H_
#define LEPP3_UTIL_H_

#include <ctime>
#include <iomanip>

namespace lepp {
  
std::string get_current_timestamp() {
  std::stringstream ss;
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );

  // Year
  ss << (now->tm_year + 1900) << '-';
  // Month
  ss << std::setw(2) << std::setfill('0') << (now->tm_mon + 1) << '-';
  // Day
  ss << std::setw(2) << std::setfill('0') << now->tm_mday << '_';
  // Hour
  ss << std::setw(2) << std::setfill('0') << now->tm_hour << '-';
  // Minute
  ss << std::setw(2) << std::setfill('0') << now->tm_min << '-';
  // Second
  ss << std::setw(2) << std::setfill('0') << now->tm_sec;

  return ss.str();
}

} // namespace lepp

#endif // LEPP3_UTIL_H_
