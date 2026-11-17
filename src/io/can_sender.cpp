#include "io/can_sender.hpp"

CanSender::CanSender(rclcpp::Node * node)
{
  pub_ = node->create_publisher<putm_vcl_interfaces::msg::LapTimer>("/putm_vcl/lap_timer", 50);
}

void CanSender::publishMetrics(const LapMetrics & metrics)
{
  auto message = putm_vcl_interfaces::msg::LapTimer();
  message.delta = static_cast<int16_t>(metrics.delta_time_s * 1000.0);
  message.current_lap = static_cast<uint16_t>(metrics.last_lap_time_ms);
  message.best_lap = static_cast<uint16_t>(metrics.best_lap_time_ms);
  message.lap_counter = metrics.lap_count;
  pub_->publish(message);
}
