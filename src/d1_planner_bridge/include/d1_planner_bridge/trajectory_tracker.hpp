#ifndef D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_
#define D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"

namespace d1_planner_bridge
{

struct TrackerParams
{
  double max_vx{0.6};
  double max_wz{1.0};
  double yaw_kp{1.5};
  double yaw_rate_ff{1.0};
  bool project_velocity_to_body{true};
  /// If |vx| is in (vx_deadband, min_vx), boost to min_vx (quadruped static friction).
  double min_vx{0.0};
  double vx_deadband{0.01};
  /// Cap on |yaw_kp * yaw_err| so turn-in-place does not dominate slow forward motion.
  double max_wz_yaw_p{1.0};

  /// Cross-track correction: pos_cmd.position vs odom (carrot on traj_server lookahead point).
  bool enable_lateral_correction{true};
  double lateral_kp{0.9};
  double lateral_error_deadband{0.05};
  double max_wz_lateral_p{0.35};
  /// Reduce |vx| when lateral error is large (0 = off).
  double vx_lat_damp_gain{0.4};
  double lateral_slowdown_dist{0.4};
};

struct GroundTwist
{
  double vx{0.0};
  double wz{0.0};
  bool valid{false};
  /// Signed lateral offset (m): robot left of path heading is positive.
  double lateral_error{0.0};
};

/// Maps EGO pos_cmd to D1 cmd_twist: velocity feedforward + yaw & lateral error feedback.
class TrajectoryTracker
{
public:
  TrajectoryTracker() = default;
  explicit TrajectoryTracker(const TrackerParams & params);

  void setParams(const TrackerParams & params);

  GroundTwist compute(
    const quadrotor_msgs::msg::PositionCommand & cmd,
    const nav_msgs::msg::Odometry * odom) const;

  geometry_msgs::msg::Twist toTwistMsg(const GroundTwist & g) const;

private:
  static double wrapPi(double a);
  static double yawFromOdom(const nav_msgs::msg::Odometry & odom);
  static double pathYawFromCmd(const quadrotor_msgs::msg::PositionCommand & cmd);
  static double signedLateralError(
    double robot_x, double robot_y, double ref_x, double ref_y, double path_yaw);

  TrackerParams params_;
};

}  // namespace d1_planner_bridge

#endif  // D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_
