#pragma once
#include "track/track_types.hpp"
#include "core/sector_tracker.hpp"
#include "io/igps_provider.hpp"
#include "utils/logger.hpp"
#include <memory>
#include <string>

class LapTimer {
public:
  explicit LapTimer(std::shared_ptr<IGpsProvider> gps_provider, bool enable_logging = true);

  bool resetGate(std::string & out_message);

  const LapMetrics & getMetrics() const {return metrics_;}
  LapState getState() const {return state_;}
  const Gate & getGate() const {return gate_;}
  std::string getLogDir() const {return logger_.getLogDirPath();}

private:
  void onGpsSample(const GpsSample & sample);
  void onSpeedSample(double speed);
  void updateHeadingAnchor(const GeoPoint & curr_pos);
  void processLapCrossing(double now_s);
  void handleRecordingReferenceLap(const GeoPoint & curr_pos, double now_s);
  void handleLapping(const GeoPoint & curr_pos, double now_s);

  std::shared_ptr<IGpsProvider> gps_provider_;
  Logger logger_;
  SectorTracker sector_tracker_;

  LapState state_ = LapState::WAITING_FOR_START;
  Gate gate_;
  LapMetrics metrics_{};

  GeoPoint current_pos_{};
  GeoPoint prev_pos_{};
  GeoPoint heading_ref_pos_{};
  double last_valid_heading_deg_ = 0.0;
  double current_spd_ = 0.0;
  double current_lap_start_time_s_ = 0.0;
};
