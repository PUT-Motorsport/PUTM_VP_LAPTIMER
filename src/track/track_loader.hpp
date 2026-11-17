#pragma once
#include "track/track_types.hpp"

class TrackLoader {
public:
  static constexpr double START_CENTER_LAT = 52.239048;
  static constexpr double START_CENTER_LON = 16.230333;
  static constexpr double START_HEADING_DEG = 292.0;
  static constexpr double DEFAULT_HALF_WIDTH_M = 10.0;
  static constexpr double RESET_GATE_OFFSET_M = 1.0;

  static Gate loadDefaultStartGate();
  static Gate calibrateGateInFrontOfCar(const GeoPoint & car_pos, double heading_deg);
};
