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
  double max_wz{0.5};
  double yaw_kp{1.5};
  double yaw_rate_ff{1.0};
  /// Cap |yaw_dot| feedforward from traj_server (avoids ±pi spikes on replan).
  double max_yaw_dot_ff{0.5};
  bool project_velocity_to_body{true};
  /// If |vx| is in (vx_deadband, min_vx), boost to min_vx (quadruped static friction).
  double min_vx{0.0};
  double vx_deadband{0.01};
  /// Cap on |yaw_kp * yaw_err| while driving (small turns during forward motion).
  double max_wz_yaw_p{0.5};
  /// |wz| floor when |heading_err| > align_heading_thresh_rad (turn-in-place).
  double min_turn_wz{0.5};

  /// Turn in place when |heading_err| exceeds this (rad); no forward/backward until aligned.
  double align_heading_thresh_rad{1.0471975511965976};  // 60 deg
  bool allow_reverse{false};

  /// Cross-track correction: pos_cmd.position vs odom (carrot on traj_server lookahead point).
  bool enable_lateral_correction{true};
  double lateral_kp{0.9};
  double lateral_error_deadband{0.05};
  double max_wz_lateral_p{0.35};
  /// Reduce |vx| when lateral error is large (0 = off).
  double vx_lat_damp_gain{0.4};
  double lateral_slowdown_dist{0.4};
  /// Skip lateral P when plan speed |v_xy| below this (stale / ended traj).
  double min_plan_speed_for_lateral{0.15};
  /// Skip lateral P when |e_lat| above this (robot far off path).
  double max_lateral_error_m{2.0};
};

struct GroundTwist
{
  double vx{0.0};
  double wz{0.0};
  bool valid{false};
  /// Signed lateral offset (m): robot left of path heading is positive.
  double lateral_error{0.0};
  /// wrapPi(path_yaw - robot_forward_yaw); forward = body +Z in global XY.
  double heading_error{0.0};
  /// Heading reference used for control (selected path tangent / carrot).
  double path_yaw{0.0};
  /// atan2(cmd.velocity) for debug comparison with path_yaw.
  double path_yaw_vel{0.0};
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

  /// Yaw of body +Z (optical / OpenVINS IMU forward) projected onto global XY.
  static double forwardYawFromOdom(const nav_msgs::msg::Odometry & odom);

private:
  struct BodyForwardHoriz
  {
    double yaw{0.0};
    double dir_x{1.0};
    double dir_y{0.0};
  };

  static double wrapPi(double a);
  static BodyForwardHoriz bodyForwardHoriz(const nav_msgs::msg::Odometry & odom);
  /// Instantaneous velocity bearing in global XY.
  static double velocityYawFromCmd(const quadrotor_msgs::msg::PositionCommand & cmd);
  /// Pick cmd.yaw vs velocity yaw (and carrot on overshoot) to minimize heading error.
  static double selectPathYaw(
    const quadrotor_msgs::msg::PositionCommand & cmd,
    double robot_yaw, double robot_x, double robot_y,
    const BodyForwardHoriz & forward);
  static double signedLateralError(
    double robot_x, double robot_y, double ref_x, double ref_y, double path_yaw);

  TrackerParams params_;
};

}  // namespace d1_planner_bridge

#endif  // D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_
