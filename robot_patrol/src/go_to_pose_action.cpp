#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class GoToPose : public rclcpp::Node {
public:
  using Pose2D = geometry_msgs::msg::Pose2D;
  using GoToPoseAct = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAct>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPoseAct>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPose::odom_topic_callback, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<GoToPoseAct>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  Pose2D desired_pos_;
  Pose2D current_pos_;

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Convert quaternion -> Euler angles
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAct::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    desired_pos_ = goal_handle->get_goal()->goal_pos;
    // Convert theta from degrees to radians
    desired_pos_.theta = desired_pos_.theta * M_PI / 180.0;

    auto result = std::make_shared<GoToPoseAct::Result>();
    auto move = geometry_msgs::msg::Twist();
    auto feedback = std::make_shared<GoToPoseAct::Feedback>();

    int mode = 0; // 0 -> Move to goal, 1 -> Rotate after reaching goal
    const double GOAL_DIST_TOLERANCE = 0.05;
    const double GOAL_THETA_TOLERANCE = 0.05;

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Current: X=%f, Y=%f, Theta=%f",
                  current_pos_.x, current_pos_.y, current_pos_.theta);
      RCLCPP_INFO(this->get_logger(), "Desired: X=%f, Y=%f, Theta=%f",
                  desired_pos_.x, desired_pos_.y, desired_pos_.theta);

      double dist_to_goal = sqrt(pow(current_pos_.y - desired_pos_.y, 2) +
                                 pow(current_pos_.x - desired_pos_.x, 2));

      if (dist_to_goal > GOAL_DIST_TOLERANCE) {
        mode = 0;
      } else {
        mode = 1;
      }
      RCLCPP_INFO(this->get_logger(), "Mode=%d, Dist to goal: %f", mode,
                  dist_to_goal);

      double vector_theta = atan2(desired_pos_.y - current_pos_.y,
                                  desired_pos_.x - current_pos_.x);
      double theta_delta = mode == 0 ? vector_theta - current_pos_.theta
                                     : desired_pos_.theta - current_pos_.theta;
      if (theta_delta > M_PI) {
        theta_delta -= 2 * M_PI;
      } else if (theta_delta < -M_PI) {
        theta_delta += 2 * M_PI;
      }
      RCLCPP_INFO(this->get_logger(), "Theta: Current=%f, Vector=%f, Delta=%f ",
                  current_pos_.theta, vector_theta, theta_delta);

      move.angular.z = theta_delta;
      move.linear.x = mode == 0 ? 0.2 : 0;
      publisher_->publish(move);
      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);

      if (mode == 1 && abs(theta_delta) < GOAL_THETA_TOLERANCE) {
        break;
      }

      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      loop_rate.sleep();
    }

    // Comment
    // for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request

    //   // Move robot forward and send feedback
    //   message = "Moving forward...";
    //   move.linear.x = 0.3;
    //   publisher_->publish(move);
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      move.linear.x = 0.0;
      move.angular.z = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}