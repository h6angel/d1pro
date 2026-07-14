#ifndef TRAJ_UTILS__ODOM_DIAGNOSTICS_HPP_
#define TRAJ_UTILS__ODOM_DIAGNOSTICS_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>

#include <rclcpp/time.hpp>

namespace traj_utils
{

/// Classify odom reception: lag (stale/skipped stamps) vs implausible jump (VIO-like).
/// Diagnostic only — no control action.
struct OdomDiagSample
{
  bool valid{false};
  double stamp_age{0.0};       // now - msg.stamp (s)
  double stamp_dt{0.0};        // msg.stamp - prev.stamp (s)
  double wall_dt{0.0};         // recv wall - prev recv wall (s)
  double pose_step_xy{0.0};    // XY displacement (m)
  double implied_speed{0.0};   // pose_step_xy / stamp_dt (m/s)
  const char * kind{"FIRST"};  // FIRST / OK / LIKELY_LAG / SUSPECT_JUMP / STAMP_BACK
};

struct OdomDiagParams
{
  bool enable{true};
  int period_ms{500};
  double implausible_speed{0.5};
  double stamp_dt_lag_sec{0.15};
  double stamp_age_warn_sec{0.10};
};

class OdomReceiveDiagnostics
{
public:
  void setParams(const OdomDiagParams & p) { params_ = p; }

  const OdomDiagParams & params() const { return params_; }

  OdomDiagSample update(
    const rclcpp::Time & stamp,
    const rclcpp::Time & now,
    double x,
    double y)
  {
    OdomDiagSample s;
    s.stamp_age = (now - stamp).seconds();
    if (!std::isfinite(s.stamp_age)) {
      s.stamp_age = 0.0;
    }

    if (!have_prev_) {
      have_prev_ = true;
      prev_stamp_ = stamp;
      prev_recv_ = now;
      prev_x_ = x;
      prev_y_ = y;
      s.valid = true;
      s.kind = "FIRST";
      return s;
    }

    s.stamp_dt = (stamp - prev_stamp_).seconds();
    s.wall_dt = (now - prev_recv_).seconds();
    s.pose_step_xy = std::hypot(x - prev_x_, y - prev_y_);
    if (!std::isfinite(s.stamp_dt)) {
      s.stamp_dt = 0.0;
    }
    if (!std::isfinite(s.wall_dt)) {
      s.wall_dt = 0.0;
    }
    if (!std::isfinite(s.pose_step_xy)) {
      s.pose_step_xy = 0.0;
    }

    if (s.stamp_dt < -1e-4) {
      s.kind = "STAMP_BACK";
      s.implied_speed = 0.0;
    } else if (s.stamp_dt < 1e-6) {
      s.kind = "OK";
      s.implied_speed = 0.0;
    } else {
      s.implied_speed = s.pose_step_xy / s.stamp_dt;
      const bool speed_high =
        s.implied_speed > params_.implausible_speed;
      const bool age_high = s.stamp_age > params_.stamp_age_warn_sec;
      // Processing backlog: wall gap much larger than stamp gap, or long stamp gap.
      const bool lag_like =
        age_high ||
        s.stamp_dt >= params_.stamp_dt_lag_sec ||
        (s.wall_dt > s.stamp_dt + 0.05 && s.wall_dt > params_.stamp_dt_lag_sec);

      if (speed_high && lag_like) {
        s.kind = "LIKELY_LAG";
      } else if (speed_high) {
        s.kind = "SUSPECT_JUMP";
      } else if (lag_like && (age_high || s.wall_dt > 0.2)) {
        s.kind = "LIKELY_LAG";
      } else {
        s.kind = "OK";
      }
    }

    prev_stamp_ = stamp;
    prev_recv_ = now;
    prev_x_ = x;
    prev_y_ = y;
    s.valid = true;

    ++count_total_;
    if (std::strcmp(s.kind, "LIKELY_LAG") == 0) {
      ++count_lag_;
    } else if (std::strcmp(s.kind, "SUSPECT_JUMP") == 0) {
      ++count_jump_;
    } else if (std::strcmp(s.kind, "STAMP_BACK") == 0) {
      ++count_stamp_back_;
    }

    last_ = s;
    return s;
  }

  /// Periodic summary; returns false if not due yet.
  bool takeSummary(
    const rclcpp::Time & now,
    uint64_t & out_total,
    uint64_t & out_lag,
    uint64_t & out_jump,
    uint64_t & out_stamp_back)
  {
    if (!params_.enable) {
      return false;
    }
    if (last_summary_.nanoseconds() == 0) {
      last_summary_ = now;
      return false;
    }
    const double dt = (now - last_summary_).seconds();
    if (dt * 1000.0 < static_cast<double>(params_.period_ms)) {
      return false;
    }
    out_total = count_total_;
    out_lag = count_lag_;
    out_jump = count_jump_;
    out_stamp_back = count_stamp_back_;
    count_total_ = count_lag_ = count_jump_ = count_stamp_back_ = 0;
    last_summary_ = now;
    return true;
  }

  const OdomDiagSample & last() const { return last_; }

private:
  OdomDiagParams params_;
  bool have_prev_{false};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_recv_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_summary_{0, 0, RCL_ROS_TIME};
  double prev_x_{0.0};
  double prev_y_{0.0};
  OdomDiagSample last_;
  uint64_t count_total_{0};
  uint64_t count_lag_{0};
  uint64_t count_jump_{0};
  uint64_t count_stamp_back_{0};
};

}  // namespace traj_utils

#endif  // TRAJ_UTILS__ODOM_DIAGNOSTICS_HPP_
