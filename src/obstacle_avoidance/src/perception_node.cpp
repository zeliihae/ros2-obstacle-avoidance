#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"

// /scan  -->  /obstacle_distance (on), /obstacle_left (sol), /obstacle_right (sag)
//
// Sektorler (robot ekseninden, saat yonunun tersi pozitif):
//   on  : [-front, +front]            (varsayilan +/- 30 derece)
//   sol : (+front, +side]             (30 ile 90 derece)
//   sag : [-side, -front)             (-90 ile -30 derece)
class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("perception_node")
  {
    front_angle_deg_ = declare_parameter<double>("front_angle_deg", 30.0);
    side_angle_deg_ = declare_parameter<double>("side_angle_deg", 90.0);
    if (side_angle_deg_ <= front_angle_deg_) {
      RCLCPP_WARN(get_logger(), "side_angle_deg (%.1f) <= front_angle_deg (%.1f): 90 derece kullaniliyor",
                  side_angle_deg_, front_angle_deg_);
      side_angle_deg_ = 90.0;
    }

    pub_ = create_publisher<std_msgs::msg::Float32>("obstacle_distance", 10);
    pub_left_ = create_publisher<std_msgs::msg::Float32>("obstacle_left", 10);
    pub_right_ = create_publisher<std_msgs::msg::Float32>("obstacle_right", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scanCallback(*msg); });

    RCLCPP_INFO(get_logger(),
                "Perception node started (front: +/- %.1f deg, sides up to +/- %.1f deg)",
                front_angle_deg_, side_angle_deg_);
  }

private:
  // [lo, hi] araligindaki (radyan) en kucuk gecerli mesafe. Engel yoksa range_max.
  static float sectorMin(const sensor_msgs::msg::LaserScan & scan, double lo, double hi)
  {
    float min_dist = scan.range_max;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      angle = std::atan2(std::sin(angle), std::cos(angle));  // [-pi, pi]
      if (angle < lo || angle > hi) {
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
    return min_dist;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan & scan)
  {
    constexpr double kDeg2Rad = M_PI / 180.0;
    constexpr double kEps = 1e-6;  // on sektor sinirindaki isinin iki sektore birden girmesini onler
    const double front = front_angle_deg_ * kDeg2Rad;
    const double side = side_angle_deg_ * kDeg2Rad;

    std_msgs::msg::Float32 out;
    out.data = sectorMin(scan, -front, front);
    pub_->publish(out);

    out.data = sectorMin(scan, front + kEps, side);
    pub_left_->publish(out);

    out.data = sectorMin(scan, -side, -front - kEps);
    pub_right_->publish(out);
  }

  double front_angle_deg_;
  double side_angle_deg_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}