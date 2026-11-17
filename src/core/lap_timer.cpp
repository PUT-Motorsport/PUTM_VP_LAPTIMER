#include "core/lap_timer.hpp"
#include "core/geo_math.hpp"
#include "track/track_loader.hpp"

LapTimer::LapTimer(std::shared_ptr<IGpsProvider> gps_provider, bool enable_logging)
: gps_provider_(std::move(gps_provider)),
  logger_(enable_logging),
  gate_(TrackLoader::loadDefaultStartGate())
{
  gps_provider_->setGpsCallback([this](const GpsSample & sample) {onGpsSample(sample);});
  gps_provider_->setSpeedCallback([this](double spd) {onSpeedSample(spd);});
}

void LapTimer::onSpeedSample(double speed)
{
  current_spd_ = speed;
}

void LapTimer::updateHeadingAnchor(const GeoPoint & curr_pos)
{
  if (heading_ref_pos_.lat == 0.0 && heading_ref_pos_.lon == 0.0) {
    heading_ref_pos_ = curr_pos;
    return;
  }
  if (GeoMath::haversineDistance(heading_ref_pos_, curr_pos) > 1.0) {
    last_valid_heading_deg_ = GeoMath::calculateBearing(heading_ref_pos_, curr_pos);
    heading_ref_pos_ = curr_pos;
  }
}

void LapTimer::onGpsSample(const GpsSample & sample)
{
  current_pos_ = sample.position;
  double now_s = sample.timestamp_s;

  updateHeadingAnchor(current_pos_);

  if (GeoMath::checkLineCrossing(prev_pos_, current_pos_, gate_)) {
    processLapCrossing(now_s);
  }

  switch (state_) {
    case LapState::WAITING_FOR_START:
      break;
    case LapState::RECORDING_REFERENCE_LAP:
      handleRecordingReferenceLap(current_pos_, now_s);
      break;
    case LapState::LAPPING:
      handleLapping(current_pos_, now_s);
      break;
  }

  prev_pos_ = current_pos_;
  logger_.logSessionData(now_s, metrics_.lap_count, current_pos_, current_spd_, state_);

  if (state_ != LapState::WAITING_FOR_START) {
    double time_into_lap_s = now_s - current_lap_start_time_s_;
    logger_.logLapData(time_into_lap_s, current_pos_, current_spd_, metrics_.delta_time_s);
  }
}

void LapTimer::processLapCrossing(double now_s)
{
  if (state_ == LapState::WAITING_FOR_START) {
    metrics_.lap_count = 1;
    current_lap_start_time_s_ = now_s;
    state_ = LapState::RECORDING_REFERENCE_LAP;
  } else {
    double completed_lap_time_s = now_s - current_lap_start_time_s_;
    metrics_.last_lap_time_ms = static_cast<uint32_t>(completed_lap_time_s * 1000.0);

    bool is_new_best = (state_ == LapState::RECORDING_REFERENCE_LAP) ||
      (state_ == LapState::LAPPING && metrics_.last_lap_time_ms < metrics_.best_lap_time_ms);

    if (state_ == LapState::RECORDING_REFERENCE_LAP) {
      state_ = LapState::LAPPING;
    }

    if (is_new_best) {
      metrics_.best_lap_time_ms = metrics_.last_lap_time_ms;
      sector_tracker_.saveReferenceAsBest();
    }

    metrics_.lap_count++;
    current_lap_start_time_s_ = now_s;
    sector_tracker_.resetCurrentLap();
  }
  logger_.startNewLapFile(metrics_.lap_count);
}

void LapTimer::handleRecordingReferenceLap(const GeoPoint & curr_pos, double now_s)
{
  double time_into_lap_s = now_s - current_lap_start_time_s_;
  sector_tracker_.recordSectorIfNeeded(curr_pos, time_into_lap_s);
}

void LapTimer::handleLapping(const GeoPoint & curr_pos, double now_s)
{
  double time_into_lap_s = now_s - current_lap_start_time_s_;
  sector_tracker_.recordSectorIfNeeded(curr_pos, time_into_lap_s);
  metrics_.delta_time_s = sector_tracker_.calculateDelta(curr_pos, time_into_lap_s);
}

bool LapTimer::resetGate(std::string & out_message)
{
  if (last_valid_heading_deg_ == 0.0 || current_pos_.lat == 0.0) {
    out_message = "Car hasn't moved yet! Drive forward 1-2 meters first.";
    return false;
  }

  gate_ = TrackLoader::calibrateGateInFrontOfCar(current_pos_, last_valid_heading_deg_);
  state_ = LapState::WAITING_FOR_START;
  metrics_ = LapMetrics{};
  sector_tracker_.resetAll();
  logger_.closeLapFile();

  out_message = "Start Line Calibrated Successfully!";
  return true;
}
