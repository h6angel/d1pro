#include "d1_planner_bridge/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace d1_planner_bridge
{

TrajectoryTracker::TrajectoryTracker(const TrackerParams & params)
: params_(params)
{
}

void TrajectoryTracker::setParams(const TrackerParams & params)
{
  params_ = params;
}

double TrajectoryTracker::wrapPi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double TrajectoryTracker::yawFromOdom(const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

GroundTwist TrajectoryTracker::compute(
  const quadrotor_msgs::msg::PositionCommand & cmd,
  const nav_msgs::msg::Odometry & odom) const
{
  GroundTwist out;

  if (cmd.trajectory_flag != quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY) {
    return out;
  }

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const double robot_yaw = yawFromOdom(odom);

  const double dx = cmd.position.x - robot_x;
  const double dy = cmd.position.y - robot_y;
  const double dist_xy = std::hypot(dx, dy);

  const double vel_xy = std::hypot(cmd.velocity.x, cmd.velocity.y);

  out.at_goal_xy =
    (dist_xy < params_.goal_xy_tolerance) &&
    (vel_xy < params_.stop_vel_threshold);

  if (out.at_goal_xy) {
    out.valid = true;
    out.vx = 0.0;
    out.wz = params_.yaw_kp * wrapPi(cmd.yaw - robot_yaw);
    out.wz = std::clamp(out.wz, -params_.max_wz, params_.max_wz);
    return out;
  }

  double vx = 0.0;
  if (params_.project_velocity_to_body) {
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    vx = cos_yaw * cmd.velocity.x + sin_yaw * cmd.velocity.y;
  } else {
    vx = cmd.velocity.x;
  }

  const double yaw_err = wrapPi(cmd.yaw - robot_yaw);
  double wz = params_.yaw_kp * yaw_err + params_.yaw_rate_ff * cmd.yaw_dot;

  vx = std::clamp(vx, -params_.max_vx, params_.max_vx);
  wz = std::clamp(wz, -params_.max_wz, params_.max_wz);

  out.vx = vx;
  out.wz = wz;
  out.valid = true;
  return out;
}

geometry_msgs::msg::Twist TrajectoryTracker::toTwistMsg(const GroundTwist & g) const
{
  // D1 /command/cmd_twist: only linear.x (forward) and angular.z (yaw rate) are actuated.
  geometry_msgs::msg::Twist twist;
  twist.linear.x = g.vx;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = g.wz;
  return twist;
}

}  // namespace d1_planner_bridge
