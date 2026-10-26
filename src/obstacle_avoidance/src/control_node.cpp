// control_node.cpp
// /state (String) + /turn_direction (Int8) --> hiz profili --> /cmd_vel (Twist)
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#include "obstacle_avoidance/avoidance_state_machine.hpp"

using namespace std::chrono_literals;
using obstacle_avoidance::State;
using obstacle_avoidance::state_from_string;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
    cruise_speed_ = std::max(0.0, declare_parameter<double>("cruise_speed", 0.4));
    slow_speed_ = std::max(0.0, declare_parameter<double>("slow_speed", 0.15));
    turn_speed_ = std::max(0.0, declare_parameter<double>("turn_speed", 0.6));
    const int dir = declare_parameter<int>("turn_direction", 1);  // varsayilan: +1 sol, -1 sag
    turn_sign_ = (dir >= 0) ? 1.0 : -1.0;
    max_accel_ = std::max(0.01, declare_parameter<double>("max_accel", 0.5));   // [m/s^2]
    max_decel_ = std::max(0.01, declare_parameter<double>("max_decel", 1.5));   // [m/s^2]
    state_timeout_ = declare_parameter<double>("state_timeout", 0.5);           // [s]
    control_rate_ = std::max(1.0, declare_parameter<double>("control_rate", 20.0));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_sub_ = create_subscription<std_msgs::msg::String>(
      "/state", 10, [this](const std_msgs::msg::String::SharedPtr msg) {on_state(*msg);});

    turn_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/turn_direction", 10, [this](const std_msgs::msg::Int8::SharedPtr msg) {
        turn_sign_ = (msg->data >= 0) ? 1.0 : -1.0;  // decision_node'un sectigi yon
      });

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {on_timer();});

    RCLCPP_INFO(
      get_logger(), "control_node basladi: cruise=%.2f slow=%.2f turn=%.2f (m/s, rad/s)",
      cruise_speed_, slow_speed_, turn_speed_);
  }

  ~ControlNode() override
  {
    // En iyi caba (best effort): kapanirken robotu durdur.
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  void on_state(const std_msgs::msg::String & msg)
  {
    bool ok = false;
    const State parsed = state_from_string(msg.data, &ok);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Bilinmeyen durum '%s' -> DUR", msg.data.c_str());
    }
    state_ = parsed;
    last_state_time_ = this->now().seconds();
    have_state_ = true;
  }

  void on_timer()
  {
    const double dt = 1.0 / control_rate_;
    const double now = this->now().seconds();

    State s = state_;
    const bool stale = !have_state_ || (now - last_state_time_) > state_timeout_;
    if (stale) {
      s = State::DUR;  // karar katmanindan haber yoksa dur
      if (!warned_) {
        RCLCPP_WARN(get_logger(), "/state gelmiyor -> DUR");
        warned_ = true;
      }
    } else {
      warned_ = false;
    }

    double target_linear = 0.0;
    double target_angular = 0.0;
    switch (s) {
      case State::SERBEST: target_linear = cruise_speed_; break;
      case State::YAVASLA: target_linear = slow_speed_; break;
      case State::KACIN:   target_angular = turn_sign_ * turn_speed_; break;  // yerinde don
      case State::DUR:     break;
    }

    if (s == State::DUR) {
      linear_ = 0.0;  // acil fren: rampa yok, aninda sifir
    } else {
      linear_ = ramp(linear_, target_linear, dt);
    }
    const double angular = target_angular;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  // Hizlanirken max_accel, yavaslarken max_decel ile sinirlanmis gecis.
  double ramp(double current, double target, double dt) const
  {
    if (target > current) {
      return std::min(current + max_accel_ * dt, target);
    }
    return std::max(current - max_decel_ * dt, target);
  }

  double cruise_speed_{0.4}, slow_speed_{0.15}, turn_speed_{0.6}, turn_sign_{1.0};
  double max_accel_{0.5}, max_decel_{1.5}, state_timeout_{0.5}, control_rate_{20.0};

  State state_{State::DUR};
  double last_state_time_{0.0};
  double linear_{0.0};
  bool have_state_{false};
  bool warned_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr turn_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}