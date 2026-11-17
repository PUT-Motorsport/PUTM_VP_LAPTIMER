#pragma once
#include "track/track_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/lap_timer.hpp"

class CanSender {
public:
  explicit CanSender(rclcpp::Node * node);
  void publishMetrics(const LapMetrics & metrics);

private:
  rclcpp::Publisher<putm_vcl_interfaces::msg::LapTimer>::SharedPtr pub_;
};
