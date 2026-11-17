#pragma once
#include <string>
#include <sstream>
#include <ctime>
#include <iomanip>

namespace TimeUtils
{

inline std::string formatCurrentTime(const char * pattern)
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(&tm, pattern);
  return ss.str();
}

inline std::string getSessionTimestampStr()
{
  return formatCurrentTime("%Y-%m-%d_%H-%M-%S");
}

inline std::string getLapTimestampStr()
{
  return formatCurrentTime("%H-%M-%S");
}

} // namespace TimeUtils
