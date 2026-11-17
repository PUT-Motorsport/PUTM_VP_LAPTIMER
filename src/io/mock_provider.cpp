#include "io/mock_provider.hpp"

void MockProvider::feedGps(const GeoPoint & pt, double timestamp_s)
{
  if (gps_cb_) {
    gps_cb_(GpsSample{pt, timestamp_s});
  }
}

void MockProvider::feedSpeed(double speed)
{
  if (speed_cb_) {
    speed_cb_(speed);
  }
}
