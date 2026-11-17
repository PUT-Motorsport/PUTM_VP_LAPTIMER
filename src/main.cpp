#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "core/lap_timer.hpp"
#include "io/gnss_driver.hpp"
#include "io/can_sender.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class LapTimerNode : public rclcpp::Node {
public:
  LapTimerNode()
  : Node("lap_timer")
  {
    gnss_driver_ = std::make_shared<GnssDriver>(this);
    can_sender_ = std::make_unique<CanSender>(this);
    lap_timer_ = std::make_unique<LapTimer>(gnss_driver_, true);

    reset_gate_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/lap_timer/reset_gate",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string msg;
        response->success = lap_timer_->resetGate(msg);
        response->message = msg;
        if (!response->success) {
          RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        }
            });

    wall_timer_ = this->create_wall_timer(20ms, [this]() {
          can_sender_->publishMetrics(lap_timer_->getMetrics());
        });

    const auto & g = lap_timer_->getGate();
    RCLCPP_INFO(this->get_logger(), "Gate Calculated: P1(%.8f, %.8f) - P2(%.8f, %.8f)",
                    g.p1.lat, g.p1.lon, g.p2.lat, g.p2.lon);
    RCLCPP_INFO(this->get_logger(), "Log directory path: %s", lap_timer_->getLogDir().c_str());
  }

private:
  std::shared_ptr<GnssDriver> gnss_driver_;
  std::unique_ptr<CanSender> can_sender_;
  std::unique_ptr<LapTimer> lap_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_gate_srv_;
  rclcpp::TimerBase::SharedPtr wall_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LapTimerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
