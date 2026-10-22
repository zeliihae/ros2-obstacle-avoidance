#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    front_angle_deg_ = declare_parameter<double>("front_angle_deg", 30.0);

    pub_ = create_publisher<std_msgs::msg::Float32>("obstacle_distance", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scanCallback(*msg); });

    RCLCPP_INFO(get_logger(), "Perception node started (front sector: +/- %.1f deg)",
                front_angle_deg_);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan & scan)
  {
    const double limit = front_angle_deg_ * M_PI / 180.0;
    float min_dist = scan.range_max;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      angle = std::atan2(std::sin(angle), std::cos(angle));  // [-pi, pi]
      if (std::fabs(angle) > limit) {
        continue;
      }

      float r = scan.ranges[i];
      if (std::isnan(r)) {
        continue;                  // gecersiz olcum
      }
      if (std::isinf(r)) {
        if (r < 0.0f) {
          r = scan.range_min;      // -inf: engel minimum menzilden bile yakin
        } else {
          continue;                // +inf: menzilde engel yok
        }
      }
      if (r < min_dist) {
        min_dist = r;
      }
    }

    std_msgs::msg::Float32 out;
    out.data = min_dist;
    pub_->publish(out);
  }

  double front_angle_deg_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
