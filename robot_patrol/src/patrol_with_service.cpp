#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <math.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using LaserScan = sensor_msgs::msg::LaserScan;
using GetDirection = custom_interfaces::srv::GetDirection;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService() : Node("patrol_with_service_node") {
    move_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PatrolWithService::scan_callback, this, _1));
    timer_ = this->create_wall_timer(
        20ms, std::bind(&PatrolWithService::timer_callback, this));
    client_ = this->create_client<GetDirection>("direction_service");
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Control Loop Started");
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    RCLCPP_INFO(this->get_logger(), "Service found...");

    if (last_laser_ != NULL) {
      auto request = std::make_shared<GetDirection::Request>();
      request->laser_data = *last_laser_;
      auto result_future = client_->async_send_request(
          request, std::bind(&PatrolWithService::response_callback, this,
                             std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Request initiated");
    }
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    RCLCPP_INFO(this->get_logger(), "Response callback");
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", result->direction.c_str());
      auto message = geometry_msgs::msg::Twist();
      if (result->direction == "forward") {
        message.linear.x = 0.1;
        message.angular.z = 0.0;
      } else if (result->direction == "left") {
        message.linear.x = 0.1;
        message.angular.z = 0.5;
      } else {
        message.linear.x = 0.1;
        message.angular.z = -0.5;
      }
      move_publisher_->publish(message);
    } else {
      RCLCPP_INFO(this->get_logger(), "Service Call Timed out");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_laser_ = msg;
  }

  LaserScan::SharedPtr last_laser_ = NULL;
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolWithService>());
  rclcpp::shutdown();
  return 0;
}