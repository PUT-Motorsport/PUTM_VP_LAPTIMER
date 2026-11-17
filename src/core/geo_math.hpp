#pragma once
#include "track/track_types.hpp"

class GeoMath {
public:
  static constexpr double EARTH_RADIUS_M = 6371000.0;

  static double degreesToRadians(double degrees);
  static double haversineDistance(const GeoPoint & p1, const GeoPoint & p2);
  static double calculateBearing(const GeoPoint & p1, const GeoPoint & p2);
  static GeoPoint offsetPoint(const GeoPoint & origin, double heading_deg, double distance_m);
  static Gate setupGatePosts(const GeoPoint & center, double heading_deg, double half_width_m);
  static double crossProduct(double ax, double ay, double bx, double by, double cx, double cy);
  static bool checkLineCrossing(
    const GeoPoint & prev_pos, const GeoPoint & curr_pos,
    const Gate & gate);
};
