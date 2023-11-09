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
    float dist = 0;
    float max_dist = 2.0;

    // Right
    for (int i = 180; i < 300; i++) {
      if (std::isinf(request->laser_data.ranges[i]) ||
          request->laser_data.ranges[i] > max_dist) {
        dist = max_dist;
      } else {
        dist = request->laser_data.ranges[i];
      }
      total_dist_sec_right += dist;
    }

    // Front
    for (int i = 300; i < 420; i++) {
      if (std::isinf(request->laser_data.ranges[i]) ||
          request->laser_data.ranges[i] > max_dist) {
        dist = max_dist;
      } else {
        dist = request->laser_data.ranges[i];
      }
      total_dist_sec_front += dist;
    }

    // Left
    for (int i = 420; i < 540; i++) {
      if (std::isinf(request->laser_data.ranges[i]) ||
          request->laser_data.ranges[i] > max_dist) {
        dist = max_dist;
      } else {
        dist = request->laser_data.ranges[i];
      }
      total_dist_sec_left += dist;
    }

    RCLCPP_INFO(this->get_logger(), "Left [%f]   Front [%f]   Right [%f]",
                total_dist_sec_left, total_dist_sec_front,
                total_dist_sec_right);

    if (total_dist_sec_right > total_dist_sec_front &&
        total_dist_sec_right > total_dist_sec_left) {
      response->direction = "right";
    } else if (total_dist_sec_front > total_dist_sec_right &&
               total_dist_sec_front > total_dist_sec_left) {
      response->direction = "forward";
    } else {
      response->direction = "left";
    }

    RCLCPP_INFO(this->get_logger(), "Direction: %s",
                response->direction.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}