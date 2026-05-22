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

double TrajectoryTracker::pathYawFromCmd(const quadrotor_msgs::msg::PositionCommand & cmd)
{
  const double vxy = std::hypot(cmd.velocity.x, cmd.velocity.y);
  if (vxy > 0.05) {
    return std::atan2(cmd.velocity.y, cmd.velocity.x);
  }
  return cmd.yaw;
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

  const double v_plan = std::hypot(cmd.velocity.x, cmd.velocity.y);
  const double path_yaw = pathYawFromCmd(cmd);

  out.lateral_error = 0.0;
  out.heading_error = 0.0;

  if (odom != nullptr && params_.project_velocity_to_body) {
    const double robot_yaw = yawFromOdom(*odom);
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    double vx = cos_yaw * cmd.velocity.x + sin_yaw * cmd.velocity.y;

    const double yaw_err_path = wrapPi(path_yaw - robot_yaw);
    out.heading_error = yaw_err_path;

    const bool heading_misaligned =
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
      const double yaw_dot_ff = std::clamp(
        cmd.yaw_dot, -params_.max_yaw_dot_ff, params_.max_yaw_dot_ff);
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
      }

      if (params_.min_vx > 0.0) {
        const double av = std::abs(vx);
        if (av > params_.vx_deadband && av < params_.min_vx) {
          vx = params_.min_vx;
        }
      }
    }

    out.vx = std::clamp(vx, -params_.max_vx, params_.max_vx);
    out.wz = std::clamp(wz, -params_.max_wz, params_.max_wz);
  } else {
    const double yaw_dot_ff = std::clamp(
      cmd.yaw_dot, -params_.max_yaw_dot_ff, params_.max_yaw_dot_ff);
    double wz = params_.yaw_rate_ff * yaw_dot_ff;
    double vx = cmd.velocity.x;
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
