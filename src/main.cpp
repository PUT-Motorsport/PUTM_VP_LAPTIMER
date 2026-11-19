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
          const auto & g = lap_timer_->getGate();
          RCLCPP_INFO(this->get_logger(), "Gate Calculated: P1(%.8f, %.8f) - P2(%.8f, %.8f)",
            g.p1.lat, g.p1.lon, g.p2.lat, g.p2.lon);
        }
      });

    wall_timer_ = this->create_wall_timer(20ms, [this]() {
      const auto & m = lap_timer_->getMetrics();
      const LapState state = lap_timer_->getState();
      can_sender_->publishMetrics(m);

      if (prev_state_ == LapState::WAITING_FOR_START && state != LapState::WAITING_FOR_START) {
        RCLCPP_INFO(this->get_logger(), "Crossed start line for the first time. Starting lap 1.");
      }

      if (m.lap_count > prev_lap_count_ && prev_lap_count_ > 0) {
        RCLCPP_INFO(this->get_logger(), "Lap %d finished. Time: %.3f s",
          prev_lap_count_, m.last_lap_time_ms / 1000.0);
      }

      if (m.best_lap_time_ms > 0 &&
        (prev_best_lap_ms_ == 0 || m.best_lap_time_ms < prev_best_lap_ms_))
      {
        RCLCPP_INFO(this->get_logger(), "New best lap recorded with time: %.3f s",
          m.best_lap_time_ms / 1000.0);
      }

      prev_state_ = state;
      prev_lap_count_ = m.lap_count;
      prev_best_lap_ms_ = m.best_lap_time_ms;

      switch (state) {
        case LapState::WAITING_FOR_START:
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Status: WAITING... Drive to the start line.");
          break;
        case LapState::RECORDING_REFERENCE_LAP:
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Lap: %d | (Recording Reference)", m.lap_count);
          break;
        case LapState::LAPPING:
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Lap: %d | Delta: %.3f s | Best: %.3f s",
            m.lap_count, m.delta_time_s, m.best_lap_time_ms / 1000.0);
          break;
      }
    });

    const auto & g = lap_timer_->getGate();
    RCLCPP_INFO(this->get_logger(), "Gate Calculated: P1(%.8f, %.8f) - P2(%.8f, %.8f)",
      g.p1.lat, g.p1.lon, g.p2.lat, g.p2.lon);
    RCLCPP_INFO(this->get_logger(), "Log directory path: %s", lap_timer_->getLogDir().c_str());
    RCLCPP_INFO(this->get_logger(), "LapTimer initialized, waiting for first start line cross.");
  }

  ~LapTimerNode() override
  {
    RCLCPP_INFO(this->get_logger(), "Log files closed safely.");
  }

private:
  std::shared_ptr<GnssDriver> gnss_driver_;
  std::unique_ptr<CanSender> can_sender_;
  std::unique_ptr<LapTimer> lap_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_gate_srv_;
  rclcpp::TimerBase::SharedPtr wall_timer_;

  LapState prev_state_ = LapState::WAITING_FOR_START;
  uint8_t prev_lap_count_ = 0;
  uint32_t prev_best_lap_ms_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LapTimerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}