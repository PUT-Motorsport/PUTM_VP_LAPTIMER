#include "utils/logger.hpp"
#include "utils/time_utils.hpp"
#include <cstdlib>
#include <iomanip>
#include <sstream>

Logger::Logger(bool enable_file_logging)
: enabled_(enable_file_logging)
{
  if (!enabled_) {return;}

  const char * home_dir = std::getenv("HOME");
  std::string base_path = home_dir ?
    std::string(home_dir) + "/PUTM_VP_LAPTIMER/csv_logs/" :
    "/tmp/PUTM_VP_LAPTIMER/csv_logs/";

  log_dir_ = std::filesystem::path(base_path);
  std::error_code ec;
  if (!std::filesystem::exists(log_dir_)) {
    std::filesystem::create_directories(log_dir_, ec);
  }

  std::string session_filename = "SESSION_TOTAL_" + TimeUtils::getSessionTimestampStr() + ".csv";
  session_file_.open(log_dir_ / session_filename);
  if (session_file_.is_open()) {
    session_file_ << std::fixed << std::setprecision(10);
    session_file_ << "timestamp,lap_count,lat,lon,speed,state\n";
  }
}

Logger::~Logger()
{
  if (lap_file_.is_open()) {lap_file_.close();}
  if (session_file_.is_open()) {session_file_.close();}
}

void Logger::startNewLapFile(uint8_t lap_count)
{
  if (!enabled_) {return;}
  closeLapFile();

  std::stringstream ss;
  ss   << "LAP_" << std::setw(2) << std::setfill('0') << static_cast<int>(lap_count)
       << "_" << TimeUtils::getLapTimestampStr() << ".csv";

  lap_file_.open(log_dir_ / ss.str());
  if (lap_file_.is_open()) {
    lap_file_ << std::fixed << std::setprecision(10);
    lap_file_ << "time_into_lap,lat,lon,speed,delta\n";
  }
}

void Logger::closeLapFile()
{
  if (lap_file_.is_open()) {
    lap_file_.close();
  }
}

void Logger::logSessionData(
  double timestamp_s, uint8_t lap_count, const GeoPoint & pos,
  double speed, LapState state)
{
  if (!enabled_ || !session_file_.is_open()) {return;}
  session_file_   << timestamp_s << ","
                  << static_cast<int>(lap_count) << ","
                  << pos.lat << "," << pos.lon << ","
                  << speed << "," << static_cast<int>(state) << "\n";
  session_file_.flush();
}

void Logger::logLapData(double time_into_lap_s, const GeoPoint & pos, double speed, double delta_s)
{
  if (!enabled_ || !lap_file_.is_open()) {return;}
  lap_file_   << time_into_lap_s << ","
              << pos.lat << "," << pos.lon << ","
              << speed << "," << delta_s << "\n";
  lap_file_.flush();
}
