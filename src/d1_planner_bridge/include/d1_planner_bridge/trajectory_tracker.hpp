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
  double yaw_kp{1.2};
  double yaw_rate_ff{1.0};
  /// |heading_err| above this: vx=0, turn in place toward path tangent.
  double align_heading_thresh_rad{0.4};
  double min_turn_wz{0.15};
  bool project_velocity_to_body{true};
  bool allow_reverse{false};
};

struct GroundTwist
{
  double vx{0.0};
  double wz{0.0};
  bool valid{false};
  double heading_error{0.0};
};

/// Maps traj_server pos_cmd to D1 cmd_twist: body-frame vx + heading P + yaw_dot feedforward.
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

  /// Yaw of body +Z (optical / OpenVINS forward) projected onto global XY (debug logging).
  static double forwardYawFromOdom(const nav_msgs::msg::Odometry & odom);

private:
  struct BodyForwardHoriz
  {
    double yaw{0.0};
    double dir_x{1.0};
    double dir_y{0.0};
  };

  static BodyForwardHoriz bodyForwardHoriz(const nav_msgs::msg::Odometry & odom);
  static double wrapPi(double a);

  TrackerParams params_;
};

}  // namespace d1_planner_bridge

#endif  // D1_PLANNER_BRIDGE__TRAJECTORY_TRACKER_HPP_
