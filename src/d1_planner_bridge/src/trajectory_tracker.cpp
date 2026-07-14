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

TrajectoryTracker::BodyForwardHoriz TrajectoryTracker::bodyForwardHoriz(
  const nav_msgs::msg::Odometry & odom)
{
  BodyForwardHoriz out;
  const auto & q = odom.pose.pose.orientation;
  const double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!std::isfinite(n2) || n2 < 1e-8) {
    return out;
  }
  const double inv_n = 1.0 / std::sqrt(n2);
  const double w = q.w * inv_n;
  const double x = q.x * inv_n;
  const double y = q.y * inv_n;
  const double z = q.z * inv_n;

  // Body +Z axis in global (OpenVINS imu / camera optical forward).
  const double zx = 2.0 * (x * z + w * y);
  const double zy = 2.0 * (y * z - w * x);
  const double horiz = std::hypot(zx, zy);
  if (!std::isfinite(horiz) || horiz < 1e-6) {
    return out;
  }
  out.dir_x = zx / horiz;
  out.dir_y = zy / horiz;
  out.yaw = std::atan2(zy, zx);
  if (!std::isfinite(out.yaw)) {
    out.yaw = 0.0;
  }
  return out;
}

double TrajectoryTracker::forwardYawFromOdom(const nav_msgs::msg::Odometry & odom)
{
  return bodyForwardHoriz(odom).yaw;
}

GroundTwist TrajectoryTracker::compute(
  const quadrotor_msgs::msg::PositionCommand & cmd,
  const nav_msgs::msg::Odometry * odom) const
{
  GroundTwist out;

  if (cmd.trajectory_flag != quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY) {
    return out;
  }

  const double vel_x = std::isfinite(cmd.velocity.x) ? cmd.velocity.x : 0.0;
  const double vel_y = std::isfinite(cmd.velocity.y) ? cmd.velocity.y : 0.0;
  const double yaw_dot = std::isfinite(cmd.yaw_dot) ? cmd.yaw_dot : 0.0;
  const double v_plan = std::hypot(vel_x, vel_y);

  double path_yaw = std::isfinite(cmd.yaw) ? cmd.yaw : 0.0;
  if (v_plan > 0.05) {
    path_yaw = std::atan2(vel_y, vel_x);
  } else if (std::isfinite(cmd.track_yaw)) {
    path_yaw = cmd.track_yaw;
  }

  double vx = vel_x;
  double wz = params_.yaw_rate_ff * yaw_dot;
  out.heading_error = 0.0;

  if (params_.project_velocity_to_body) {
    if (odom == nullptr) {
      return out;
    }
    const BodyForwardHoriz forward = bodyForwardHoriz(*odom);
    if (!std::isfinite(forward.yaw) || !std::isfinite(path_yaw)) {
      return out;
    }

    const double yaw_err = wrapPi(path_yaw - forward.yaw);
    out.heading_error = yaw_err;

    vx = forward.dir_x * vel_x + forward.dir_y * vel_y;
    wz += params_.yaw_kp * yaw_err;

    const bool heading_misaligned =
      std::abs(yaw_err) > params_.align_heading_thresh_rad;
    if (heading_misaligned) {
      vx = 0.0;
      wz = params_.yaw_kp * yaw_err;
      if (std::abs(yaw_err) > 0.15 && std::abs(wz) < params_.min_turn_wz) {
        wz = (yaw_err > 0.0 ? 1.0 : -1.0) * params_.min_turn_wz;
      }
    }
  }

  if (!params_.allow_reverse) {
    vx = std::max(0.0, vx);
  }

  out.vx = std::clamp(vx, -params_.max_vx, params_.max_vx);
  out.wz = std::clamp(wz, -params_.max_wz, params_.max_wz);

  if (!std::isfinite(out.vx) || !std::isfinite(out.wz)) {
    out.vx = 0.0;
    out.wz = 0.0;
  }

  out.valid = true;
  return out;
}

geometry_msgs::msg::Twist TrajectoryTracker::toTwistMsg(const GroundTwist & g) const
{
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
