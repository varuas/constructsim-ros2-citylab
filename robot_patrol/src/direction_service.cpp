#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

using GetDirection = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {

public:
  DirectionService() : Node("direction_service_node") {
    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::direction_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void
  direction_callback(const std::shared_ptr<GetDirection::Request> request,
                     const std::shared_ptr<GetDirection::Response> response) {

    // RCLCPP_INFO(get_logger(), "Ranges %f", request->laser_data.ranges[0]);

    float total_dist_sec_right = 0.0;
    float total_dist_sec_front = 0.0;
    float total_dist_sec_left = 0.0;

    // Right
    for (int i = 180; i < 300; i++) {
      if (std::isinf(request->laser_data.ranges[i])) {
        total_dist_sec_right += request->laser_data.range_max;
      } else {
        total_dist_sec_right += request->laser_data.ranges[i];
      }
    }

    // Front
    for (int i = 300; i < 420; i++) {
      if (std::isinf(request->laser_data.ranges[i])) {
        total_dist_sec_front += request->laser_data.range_max;
      } else {
        total_dist_sec_front += request->laser_data.ranges[i];
      }
    }

    // Left
    for (int i = 420; i < 540; i++) {
      if (std::isinf(request->laser_data.ranges[i])) {
        total_dist_sec_left += request->laser_data.range_max;
      } else {
        total_dist_sec_left += request->laser_data.ranges[i];
      }
    }

    if (total_dist_sec_right > total_dist_sec_front &&
        total_dist_sec_right > total_dist_sec_left) {
      response->direction = "right";
    } else if (total_dist_sec_front > total_dist_sec_right &&
               total_dist_sec_front > total_dist_sec_left) {
      response->direction = "forward";
    } else {
      response->direction = "left";
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}