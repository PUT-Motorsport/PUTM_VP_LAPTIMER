#pragma once
#include "track/track_types.hpp"
#include <filesystem>
#include <fstream>
#include <string>

class Logger {
public:
  explicit Logger(bool enable_file_logging = true);
  ~Logger();

  void startNewLapFile(uint8_t lap_count);
  void closeLapFile();
  void logSessionData(
    double timestamp_s, uint8_t lap_count, const GeoPoint & pos, double speed,
    LapState state);
  void logLapData(double time_into_lap_s, const GeoPoint & pos, double speed, double delta_s);

  std::string getLogDirPath() const {return log_dir_.string();}

private:
  bool enabled_ = true;
  std::filesystem::path log_dir_;
  std::ofstream session_file_;
  std::ofstream lap_file_;
};
