#pragma once
#include "track/track_types.hpp"
#include <array>
#include <cstddef>

class SectorTracker {
public:
  static constexpr size_t MAX_SECTORS = 10000;
  static constexpr double SECTOR_RECORDING_DISTANCE_M = 0.5;
  static constexpr double MAX_MATCH_DISTANCE_M = 20.0;

  bool recordSectorIfNeeded(const GeoPoint & pos, double time_into_lap_s);
  double calculateDelta(const GeoPoint & pos, double current_time_into_lap_s) const;
  void saveReferenceAsBest();
  void resetCurrentLap();
  void resetAll();

  size_t getReferenceCount() const {return ref_sectors_count_;}
  size_t getBestCount() const {return best_sectors_count_;}

private:
  std::array<Sector, MAX_SECTORS> reference_lap_sectors_{};
  size_t ref_sectors_count_ = 0;

  std::array<Sector, MAX_SECTORS> best_lap_sectors_{};
  size_t best_sectors_count_ = 0;
};
