#ifndef D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_
#define D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_

#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"

namespace d1_planner_bridge
{

struct TrackerParams
{
  double max_vx{0.6};
  double max_wz{1.0};
  double goal_xy_tolerance{0.25};
  double stop_vel_threshold{0.05};
  double yaw_kp{1.5};
  double yaw_rate_ff{1.0};
  bool project_velocity_to_body{true};
};

struct GroundTwist
{
  double vx{0.0};
  double wz{0.0};
  bool valid{false};
  bool at_goal_xy{false};
};

/// Converts EGO PositionCommand + odom into ground-plane D1 commands (vx, wz).
/// All goal / error checks use XY only; Z is ignored.
class TrajectoryTracker
{
public:
  TrajectoryTracker() = default;
  explicit TrajectoryTracker(const TrackerParams & params);

  void setParams(const TrackerParams & params);

  /// @param cmd Latest EGO pos_cmd (world frame).
  /// @param odom Current robot odometry (world frame).
  GroundTwist compute(
    const quadrotor_msgs::msg::PositionCommand & cmd,
    const nav_msgs::msg::Odometry & odom) const;

  geometry_msgs::msg::Twist toTwistMsg(const GroundTwist & g) const;

private:
  static double wrapPi(double a);
  static double yawFromOdom(const nav_msgs::msg::Odometry & odom);

  TrackerParams params_;
};

}  // namespace d1_planner_bridge

#endif  // D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_
