#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    move_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Patrol::scan_callback, this, _1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "START -------->");
    int largest_index = 0;
    float largest_dist = msg->ranges[largest_index];
    RCLCPP_INFO(this->get_logger(), "Initial largest_dist %f", largest_dist);
    if (isinf(largest_dist)) {
      largest_dist = 0;
      RCLCPP_INFO(this->get_logger(), "Initially infinity, setting to zero");
    }

    for (int i = 1; i <= 180; i++) {
      if (isinf(msg->ranges[i])) {
        continue;
      }
      if (msg->ranges[i] > largest_dist) {
        largest_index = i;
        largest_dist = msg->ranges[i];
      }
    }
    for (int i = 540; i <= 719; i++) {
      if (isinf(msg->ranges[i])) {
        continue;
      }
      if (msg->ranges[i] > largest_dist) {
        largest_index = i;
        largest_dist = msg->ranges[i];
      }
    }

    if (largest_index < 360) {
      RCLCPP_INFO(this->get_logger(), "index less than 360");
      direction_ = (largest_index / 360.0) * M_PI;
    } else {
      RCLCPP_INFO(this->get_logger(), "index greater than 360");
      direction_ = -1.0 * (720 - largest_index) / 360 * M_PI;
    }

    RCLCPP_INFO(this->get_logger(), "Idx: %d, Dist: %f, Direction: %f",
                largest_index, largest_dist, direction_);

    // auto message = geometry_msgs::msg::Twist();
    // message.linear.x = 0.1;
    // message.angular.z = 0.2;
    // move_publisher_->publish(message);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_publisher_;
  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}