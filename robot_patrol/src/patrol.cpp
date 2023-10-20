#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <math.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    move_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Patrol::scan_callback, this, _1));
    timer_ =
        this->create_wall_timer(5ms, std::bind(&Patrol::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.1;
    message.angular.z = direction_ / 2.0;
    move_publisher_->publish(message);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    int largest_index = 180;
    float largest_dist = 0;
    int smallest_index = 180;
    float smallest_dist = 99;

    for (int i = 180; i <= 540; i++) {
      if (isinf(msg->ranges[i])) {
        continue;
      }
      if (msg->ranges[i] > largest_dist) {
        largest_index = i;
        largest_dist = msg->ranges[i];
      }
      if (msg->ranges[i] < smallest_dist) {
        smallest_index = i;
        smallest_dist = msg->ranges[i];
      }
    }

    if (smallest_dist > 0.25) {
      // Turn towards the farthest object
      direction_ = (largest_index / 720.0) * 2 * M_PI - M_PI;
    } else {
      // obstacle right in front, make a hard turn
      direction_ = smallest_index > 360 ? -M_PI / 2 : M_PI / 2;
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}