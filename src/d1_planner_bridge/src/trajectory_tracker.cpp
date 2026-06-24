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

double TrajectoryTracker::velocityYawFromCmd(
  const quadrotor_msgs::msg::PositionCommand & cmd)
{
  const double vx = std::isfinite(cmd.velocity.x) ? cmd.velocity.x : 0.0;
  const double vy = std::isfinite(cmd.velocity.y) ? cmd.velocity.y : 0.0;
  const double vxy = std::hypot(vx, vy);
  if (vxy < 1e-6) {
    return 0.0;
  }
  const double yaw = std::atan2(vy, vx);
  return std::isfinite(yaw) ? yaw : 0.0;
}

double TrajectoryTracker::selectPathYaw(
  const quadrotor_msgs::msg::PositionCommand & cmd,
  double robot_yaw, double robot_x, double robot_y,
  const BodyForwardHoriz & forward)
{
  constexpr double kOvershootVelDot = -0.02;
  constexpr double kMinCarrotDist = 0.05;

  const double vel_x = std::isfinite(cmd.velocity.x) ? cmd.velocity.x : 0.0;
  const double vel_y = std::isfinite(cmd.velocity.y) ? cmd.velocity.y : 0.0;
  const double vxy = std::hypot(vel_x, vel_y);
  const double vel_yaw = velocityYawFromCmd(cmd);
  const double cmd_yaw = std::isfinite(cmd.yaw) ? cmd.yaw : vel_yaw;
  const double vx_body = forward.dir_x * vel_x + forward.dir_y * vel_y;

  // Overshoot: plan velocity points backward — steer toward carrot, not rear tangent.
  if (vx_body < kOvershootVelDot && vxy > 1e-3) {
    const double dx = cmd.position.x - robot_x;
    const double dy = cmd.position.y - robot_y;
    if (std::hypot(dx, dy) > kMinCarrotDist) {
      const double carrot_yaw = std::atan2(dy, dx);
      if (std::isfinite(carrot_yaw)) {
        return carrot_yaw;
      }
    }
  }

  if (vxy < 1e-3) {
    return std::isfinite(cmd.yaw) ? cmd_yaw : vel_yaw;
  }

  const double err_cmd = std::abs(wrapPi(cmd_yaw - robot_yaw));
  const double err_vel = std::abs(wrapPi(vel_yaw - robot_yaw));
  return (err_vel < err_cmd) ? vel_yaw : cmd_yaw;
}

double TrajectoryTracker::signedLateralError(
  double robot_x, double robot_y, double ref_x, double ref_y, double path_yaw)
{
  const double ex = robot_x - ref_x;
  const double ey = robot_y - ref_y;
  const double ux = std::cos(path_yaw);
  const double uy = std::sin(path_yaw);
  return uy * ex - ux * ey;
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
  const double v_plan = std::hypot(vel_x, vel_y);
  out.path_yaw_vel = velocityYawFromCmd(cmd);

  out.lateral_error = 0.0;
  out.heading_error = 0.0;

  if (odom != nullptr && params_.project_velocity_to_body) {
    const BodyForwardHoriz forward = bodyForwardHoriz(*odom);
    const double robot_yaw = forward.yaw;
    const auto & pos = odom->pose.pose.position;
    const double path_yaw = selectPathYaw(cmd, robot_yaw, pos.x, pos.y, forward);
    out.path_yaw = path_yaw;
    if (!std::isfinite(robot_yaw) || !std::isfinite(path_yaw)) {
      return out;
    }

    double vx = forward.dir_x * vel_x + forward.dir_y * vel_y;

    const double yaw_err_path = wrapPi(path_yaw - robot_yaw);
    out.heading_error = yaw_err_path;

    const bool heading_misaligned =
      std::isfinite(yaw_err_path) &&
      std::abs(yaw_err_path) > params_.align_heading_thresh_rad;
    const bool plan_speed_ok = v_plan >= params_.min_plan_speed_for_lateral;

    double wz = 0.0;
    if (heading_misaligned) {
      // Turn in place toward path tangent: use full max_wz, not max_wz_yaw_p.
      wz = params_.yaw_kp * yaw_err_path;
      wz = std::clamp(wz, -params_.max_wz, params_.max_wz);
      if (std::abs(yaw_err_path) > 0.15 && std::abs(wz) < params_.min_turn_wz) {
        wz = (yaw_err_path > 0.0 ? 1.0 : -1.0) * params_.min_turn_wz;
      }
      vx = 0.0;
    } else {
      const double yaw_dot_raw = std::isfinite(cmd.yaw_dot) ? cmd.yaw_dot : 0.0;
      const double yaw_dot_ff = std::clamp(
        yaw_dot_raw, -params_.max_yaw_dot_ff, params_.max_yaw_dot_ff);
      wz = params_.yaw_rate_ff * yaw_dot_ff;

      const double wz_yaw = params_.yaw_kp * yaw_err_path;
      wz += std::clamp(wz_yaw, -params_.max_wz_yaw_p, params_.max_wz_yaw_p);

      if (params_.enable_lateral_correction && plan_speed_ok) {
        const auto & p = odom->pose.pose.position;
        out.lateral_error = signedLateralError(
          p.x, p.y, cmd.position.x, cmd.position.y, path_yaw);

        if (std::abs(out.lateral_error) <= params_.max_lateral_error_m) {
          double e_lat = out.lateral_error;
          if (std::abs(e_lat) < params_.lateral_error_deadband) {
            e_lat = 0.0;
          }

          const double wz_lat = -params_.lateral_kp * e_lat;
          wz += std::clamp(wz_lat, -params_.max_wz_lateral_p, params_.max_wz_lateral_p);

          if (params_.vx_lat_damp_gain > 0.0 && params_.lateral_slowdown_dist > 1e-3) {
            const double lat_ratio = std::min(
              std::abs(out.lateral_error) / params_.lateral_slowdown_dist, 1.0);
            const double scale = 1.0 - params_.vx_lat_damp_gain * lat_ratio;
            vx *= std::max(0.0, scale);
          }
        }
      }

      if (!params_.allow_reverse) {
        vx = std::max(0.0, vx);
      } else if (vx < 0.0 && params_.min_vx > 0.0) {
        const double av = std::abs(vx);
        if (av > params_.vx_deadband && av < params_.min_vx) {
          vx = -params_.min_vx;
        }
      }

      if (params_.min_vx > 0.0 && vx >= 0.0) {
        const double av = std::abs(vx);
        if (av > params_.vx_deadband && av < params_.min_vx) {
          vx = params_.min_vx;
        }
      }
    }

    out.vx = std::clamp(vx, -params_.max_vx, params_.max_vx);
    out.wz = std::clamp(wz, -params_.max_wz, params_.max_wz);
  } else {
    const double path_yaw = std::isfinite(cmd.yaw) ? cmd.yaw : out.path_yaw_vel;
    out.path_yaw = path_yaw;
    const double yaw_dot_raw = std::isfinite(cmd.yaw_dot) ? cmd.yaw_dot : 0.0;
    const double yaw_dot_ff = std::clamp(
      yaw_dot_raw, -params_.max_yaw_dot_ff, params_.max_yaw_dot_ff);
    double wz = params_.yaw_rate_ff * yaw_dot_ff;
    double vx = vel_x;
    if (!params_.allow_reverse) {
      vx = std::max(0.0, vx);
    }
    if (params_.min_vx > 0.0) {
      const double av = std::abs(vx);
      if (av > params_.vx_deadband && av < params_.min_vx) {
        vx = (vx >= 0.0 ? 1.0 : -1.0) * params_.min_vx;
      }
    }
    out.vx = std::clamp(vx, -params_.max_vx, params_.max_vx);
    out.wz = std::clamp(wz, -params_.max_wz, params_.max_wz);
  }

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
