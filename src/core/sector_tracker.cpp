#include "core/sector_tracker.hpp"
#include "core/geo_math.hpp"
#include <algorithm>

bool SectorTracker::recordSectorIfNeeded(const GeoPoint & pos, double time_into_lap_s)
{
  bool should_record = false;
  if (ref_sectors_count_ == 0) {
    should_record = true;
  } else {
    const auto & last = reference_lap_sectors_[ref_sectors_count_ - 1];
    if (GeoMath::haversineDistance(pos, {last.lat, last.lon}) >= SECTOR_RECORDING_DISTANCE_M) {
      should_record = true;
    }
  }

  if (should_record) {
    if (ref_sectors_count_ < MAX_SECTORS) {
      reference_lap_sectors_[ref_sectors_count_++] = {pos.lat, pos.lon, time_into_lap_s};
      return true;
    }
    return false;     // Pamięć sektorów pełna
  }
  return true;
}

double SectorTracker::calculateDelta(const GeoPoint & pos, double current_time_into_lap_s) const
{
  if (best_sectors_count_ == 0) {return 0.0;}

  double min_dist = 100.0;
  int closest_idx = -1;

  for (size_t i = 0; i < best_sectors_count_; ++i) {
    double d = GeoMath::haversineDistance(pos,
        {best_lap_sectors_[i].lat, best_lap_sectors_[i].lon});
    if (d < min_dist) {
      min_dist = d;
      closest_idx = static_cast<int>(i);
    }
  }

  if (closest_idx != -1 && min_dist < MAX_MATCH_DISTANCE_M) {
    return current_time_into_lap_s - best_lap_sectors_[closest_idx].time_s;
  }
  return 0.0;
}

void SectorTracker::saveReferenceAsBest()
{
  best_sectors_count_ = ref_sectors_count_;
  std::copy(reference_lap_sectors_.begin(),
              reference_lap_sectors_.begin() + ref_sectors_count_,
              best_lap_sectors_.begin());
}

void SectorTracker::resetCurrentLap()
{
  ref_sectors_count_ = 0;
}

void SectorTracker::resetAll()
{
  ref_sectors_count_ = 0;
  best_sectors_count_ = 0;
}
