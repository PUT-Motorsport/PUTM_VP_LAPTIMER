#include "track/track_loader.hpp"
#include "core/geo_math.hpp"

Gate TrackLoader::loadDefaultStartGate()
{
  GeoPoint center{START_CENTER_LAT, START_CENTER_LON};
  return GeoMath::setupGatePosts(center, START_HEADING_DEG, DEFAULT_HALF_WIDTH_M);
}

Gate TrackLoader::calibrateGateInFrontOfCar(const GeoPoint & car_pos, double heading_deg)
{
  GeoPoint gate_center = GeoMath::offsetPoint(car_pos, heading_deg, RESET_GATE_OFFSET_M);
  return GeoMath::setupGatePosts(gate_center, heading_deg, DEFAULT_HALF_WIDTH_M);
}
