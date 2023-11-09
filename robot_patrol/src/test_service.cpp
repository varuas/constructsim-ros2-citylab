#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;
using LaserScan = sensor_msgs::msg::LaserScan;
using GetDirection = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class TestService : public rclcpp::Node {
private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;

  void scan_callback(const LaserScan::SharedPtr msg) {
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

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;
    auto result_future = client_->async_send_request(
        request, std::bind(&TestService::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %s", result->direction.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Service Call Timed out");
    }
  }

public:
  TestService() : Node("test_direction_service") {
    client_ = this->create_client<GetDirection>("direction_service");
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TestService::scan_callback, this, _1));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestService>());
  rclcpp::shutdown();
  return 0;
}