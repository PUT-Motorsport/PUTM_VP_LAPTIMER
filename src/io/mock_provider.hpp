#pragma once
#include "io/igps_provider.hpp"

class MockProvider : public IGpsProvider {
public:
  void setGpsCallback(GpsCallback cb) override {gps_cb_ = std::move(cb);}
  void setSpeedCallback(SpeedCallback cb) override {speed_cb_ = std::move(cb);}

  void feedGps(const GeoPoint & pt, double timestamp_s);
  void feedSpeed(double speed);

private:
  GpsCallback gps_cb_;
  SpeedCallback speed_cb_;
};
