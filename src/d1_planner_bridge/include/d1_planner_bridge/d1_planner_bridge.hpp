#ifndef D1_PLANNER_BRIDGE__D1_PLANNER_BRIDGE_HPP_
#define D1_PLANNER_BRIDGE__D1_PLANNER_BRIDGE_HPP_

#include <memory>
#include <mutex>

#include "d1_planner_bridge/trajectory_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace d1_planner_bridge
{

/// Subscribes to EGO pos_cmd + odom, publishes geometry_msgs/Twist on /command/cmd_twist
/// (linear.x forward, angular.z yaw rate; other components are zero).
/// Gait / mode switching is NOT handled here (done in your D1 stack).
class D1PlannerBridgeNode : public rclcpp::Node
{
public:
  explicit D1PlannerBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlTimerCallback();

  TrackerParams loadTrackerParams();

  std::mutex mutex_;
  quadrotor_msgs::msg::PositionCommand::SharedPtr last_pos_cmd_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  rclcpp::Time last_pos_cmd_time_{0, 0, RCL_ROS_TIME};

  TrajectoryTracker tracker_;

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string pos_cmd_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  double pos_cmd_timeout_sec_{0.5};
};

}  // namespace d1_planner_bridge

#endif  // D1_PLANNER_BRIDGE__D1_PLANNER_BRIDGE_HPP_
