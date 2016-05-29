#ifndef LEPP3_UTIL_H_
#define LEPP3_UTIL_H_

#include <ctime>

namespace lepp {
  
std::string get_current_timestamp() {
  std::stringstream ss;
  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );

  // Year
  ss << (now->tm_year + 1900) << '-';
  // Month
  if (0 < now->tm_mon + 1 && now->tm_mon + 1 < 10)
    ss << "0" << now->tm_mon + 1 << '-';
  else
    ss << now->tm_mon + 1 << '-';
  // Day
  if (0 < now->tm_mday && now->tm_mday < 10)
    ss << "0" << now->tm_mday << '_';
  else
    ss << now->tm_mday << '_';
  // Hour
  if (0 < now->tm_hour && now->tm_hour < 10)
    ss << "0" << now->tm_hour;
  else
    ss << now->tm_hour;
  // Minute
  if (0 < now->tm_min && now->tm_min < 10)
    ss << "0" << now->tm_min;
  else
    ss << now->tm_min;
  // Second
  if (0 < now->tm_sec && now->tm_sec < 10)
    ss << "0" << now->tm_sec;
  else
    ss << now->tm_sec;

  return ss.str();
}

} // namespace lepp

#endif // LEPP3_UTIL_H_
