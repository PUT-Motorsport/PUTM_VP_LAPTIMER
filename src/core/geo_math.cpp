#include "core/geo_math.hpp"
#include <cmath>

double GeoMath::degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double GeoMath::haversineDistance(const GeoPoint & p1, const GeoPoint & p2)
{
  double dLat = degreesToRadians(p2.lat - p1.lat);
  double dLon = degreesToRadians(p2.lon - p1.lon);
  double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
    std::cos(degreesToRadians(p1.lat)) * std::cos(degreesToRadians(p2.lat)) *
    std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return EARTH_RADIUS_M * c;
}

double GeoMath::calculateBearing(const GeoPoint & p1, const GeoPoint & p2)
{
  double dLon = degreesToRadians(p2.lon - p1.lon);
  double y = std::sin(dLon) * std::cos(degreesToRadians(p2.lat));
  double x = std::cos(degreesToRadians(p1.lat)) * std::sin(degreesToRadians(p2.lat)) -
    std::sin(degreesToRadians(p1.lat)) * std::cos(degreesToRadians(p2.lat)) * std::cos(dLon);
  double brng = std::atan2(y, x) * 180.0 / M_PI;
  return (brng < 0.0) ? (brng + 360.0) : brng;
}

GeoPoint GeoMath::offsetPoint(const GeoPoint & origin, double heading_deg, double distance_m)
{
  double lat_rad = degreesToRadians(origin.lat);
  double hdg_rad = degreesToRadians(heading_deg);
  GeoPoint result;
  result.lat = origin.lat + (distance_m * std::cos(hdg_rad) / EARTH_RADIUS_M) * (180.0 / M_PI);
  result.lon = origin.lon +
    (distance_m * std::sin(hdg_rad) / (EARTH_RADIUS_M * std::cos(lat_rad))) * (180.0 / M_PI);
  return result;
}

Gate GeoMath::setupGatePosts(const GeoPoint & center, double heading_deg, double half_width_m)
{
  Gate gate;
  gate.p1 = offsetPoint(center, heading_deg - 90.0, half_width_m);
  gate.p2 = offsetPoint(center, heading_deg + 90.0, half_width_m);
  return gate;
}

double GeoMath::crossProduct(double ax, double ay, double bx, double by, double cx, double cy)
{
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool GeoMath::checkLineCrossing(
  const GeoPoint & prev_pos, const GeoPoint & curr_pos,
  const Gate & gate)
{
  if (prev_pos.lat == 0.0 && prev_pos.lon == 0.0) {return false;}

  double cp1 = crossProduct(gate.p1.lon, gate.p1.lat, gate.p2.lon, gate.p2.lat, prev_pos.lon,
    prev_pos.lat);
  double cp2 = crossProduct(gate.p1.lon, gate.p1.lat, gate.p2.lon, gate.p2.lat, curr_pos.lon,
    curr_pos.lat);
  double cp3 = crossProduct(prev_pos.lon, prev_pos.lat, curr_pos.lon, curr_pos.lat, gate.p1.lon,
    gate.p1.lat);
  double cp4 = crossProduct(prev_pos.lon, prev_pos.lat, curr_pos.lon, curr_pos.lat, gate.p2.lon,
    gate.p2.lat);

  return  ((cp1 > 0.0 && cp2 < 0.0) || (cp1 < 0.0 && cp2 > 0.0)) &&
         ((cp3 > 0.0 && cp4 < 0.0) || (cp3 < 0.0 && cp4 > 0.0));
}
