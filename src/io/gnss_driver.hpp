#pragma once
#include "io/igps_provider.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class GnssDriver : public IGpsProvider {
public:
  explicit GnssDriver(rclcpp::Node * node);

  void setGpsCallback(GpsCallback cb) override {gps_cb_ = std::move(cb);}
  void setSpeedCallback(SpeedCallback cb) override {speed_cb_ = std::move(cb);}

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  rclcpp::Node * node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  GpsCallback gps_cb_;
  SpeedCallback speed_cb_;
};
