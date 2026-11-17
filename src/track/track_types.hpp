#pragma once
#include <cstdint>

enum class LapState : int
{
  WAITING_FOR_START = 0,
  RECORDING_REFERENCE_LAP = 1,
  LAPPING = 2
};

struct GeoPoint
{
  double lat = 0.0;
  double lon = 0.0;
};

struct GpsSample
{
  GeoPoint position{};
  double timestamp_s = 0.0;
};

struct Gate
{
  GeoPoint p1{};
  GeoPoint p2{};
};

struct Sector
{
  double lat = 0.0;
  double lon = 0.0;
  double time_s = 0.0;
};

struct LapMetrics
{
  uint8_t lap_count = 0;
  uint32_t last_lap_time_ms = 0;
  uint32_t best_lap_time_ms = 0;
  double delta_time_s = 0.0;
};
