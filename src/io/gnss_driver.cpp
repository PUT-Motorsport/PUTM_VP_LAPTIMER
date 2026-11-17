#include "io/gnss_driver.hpp"

GnssDriver::GnssDriver(rclcpp::Node * node)
: node_(node)
{
  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/vectornav/gnss", 50,
        std::bind(&GnssDriver::gpsCallback, this, std::placeholders::_1));

  vel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body", 10,
        std::bind(&GnssDriver::velocityCallback, this, std::placeholders::_1));
}

void GnssDriver::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (gps_cb_) {
    double now_s = node_->get_clock()->now().seconds();
    gps_cb_(GpsSample{{msg->latitude, msg->longitude}, now_s});
  }
}

void GnssDriver::velocityCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  if (speed_cb_) {
    speed_cb_(msg->twist.twist.linear.x);
  }
}
