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
        this->create_wall_timer(50ms, std::bind(&Patrol::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    // message.linear.x = 0;
    // message.angular.z = 0;
    message.linear.x = 0.1;
    message.angular.z = direction_ / 2.0;
    move_publisher_->publish(message);
    // RCLCPP_INFO(this->get_logger(), "Velocity: Lin:%f, Ang:%f",
    //             message.linear.x, message.angular.z);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "START -------->");
    int largest_index = 360;
    float largest_dist = msg->ranges[largest_index];
    RCLCPP_INFO(this->get_logger(), "Initial largest_dist %f", largest_dist);
    // if (isinf(largest_dist)) {
    //   largest_dist = 0;
    //   RCLCPP_INFO(this->get_logger(), "Initially infinity, setting to zero");
    // }

    int left_index = 0;
    int right_index = 0;

    for (int i = 0; i < 180; i++) {
      left_index = 360 - i;
      if (msg->ranges[left_index] > largest_dist) {
        largest_index = left_index;
        largest_dist = msg->ranges[left_index];
      }

      right_index = 360 + i;
      if (msg->ranges[right_index] > largest_dist) {
        largest_index = right_index;
        largest_dist = msg->ranges[right_index];
      }
    }

    // if (msg->ranges[360] >= 1.5) {
    //   RCLCPP_INFO(this->get_logger(), "Front is clear!");
    //   direction_ = 0;
    // } else {
    direction_ = (largest_index / 720.0) * 2 * M_PI - M_PI;
    // }

    // if (largest_index < 360) {
    //   RCLCPP_INFO(this->get_logger(), "index less than 360");
    //   direction_ = (largest_index / 360.0) * M_PI;
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "index greater than 360");
    //   direction_ = -1.0 * (720 - largest_index) / 360 * M_PI;
    // }

    RCLCPP_INFO(this->get_logger(), "Idx: %d, Dist: %f, Direction: %f",
                largest_index, largest_dist, direction_);
    RCLCPP_INFO(this->get_logger(), "Left: %f, Middle: %f, Right: %f",
                msg->ranges[540], msg->ranges[360], msg->ranges[180]);
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