#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory =
  control_msgs::action::FollowJointTrajectory;

class InputShapingClient : public rclcpp::Node
{
public:
  using GoalHandle =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  InputShapingClient()
  : Node("input_shaping_client")
  {
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this,
      "/joint_trajectory_controller/follow_joint_trajectory"
    );

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&InputShapingClient::send_goal, this)
    );
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_sent_ = false;

  void send_goal()
  {
    if (goal_sent_) {
      return;
    }

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "FollowJointTrajectory action server not available");
      return;
    }

    FollowJointTrajectory::Goal goal;

    // ---- IMPORTANT ----
    // These joint names MUST exactly match the controller configuration
    goal.trajectory.joint_names = {
      "joint1",
      "joint2",
      "joint3"
    };

    // ---- Trajectory points (velocity-shaped input) ----
    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
    p0.velocities = {0.0, 0.0, 0.0};

    trajectory_msgs::msg::JointTrajectoryPoint p1;
    p1.time_from_start = rclcpp::Duration::from_seconds(0.5);
    p1.velocities = {1.0, 0.8, 0.6};

    trajectory_msgs::msg::JointTrajectoryPoint p2;
    p2.time_from_start = rclcpp::Duration::from_seconds(1.2);
    p2.velocities = {0.4, 0.3, 0.2};

    trajectory_msgs::msg::JointTrajectoryPoint p3;
    p3.time_from_start = rclcpp::Duration::from_seconds(2.0);
    p3.velocities = {0.0, 0.0, 0.0};

    goal.trajectory.points = {p0, p1, p2, p3};

    // Optional tolerance
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(0.2);

    auto send_goal_options =
      rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.result_callback =
      std::bind(&InputShapingClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal, send_goal_options);

    goal_sent_ = true;
    RCLCPP_INFO(get_logger(), "Input-shaped trajectory sent");
  }

  void result_callback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Trajectory execution succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Trajectory execution aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Trajectory execution canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputShapingClient>());
  return 0;
}
