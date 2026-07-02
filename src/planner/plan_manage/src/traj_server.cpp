#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/trajectory_debug_log.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <fstream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exec_bspline_path_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

double last_yaw_{0.0};
double last_yaw_dot_{0.0};
double time_forward_{0.7};

// Odom-synced playback (for slow ground robots / D1)
bool use_odom_progress_ = false;
bool have_odom_ = false;
Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
double odom_yaw_ = 0.0;
double t_progress_ = 0.0;
double odom_lookahead_time_ = 0.4;
/// XY distance to traj end at/below which vel is zeroed (align with fsm goal_reach_thresh).
double endpoint_stop_dist_ = 0.3;
double max_yaw_dot_ = 0.5;

rclcpp::Node::SharedPtr g_node;
int g_log_trace_period_ms_ = 500;

namespace
{

class TrackingTraceCsv
{
public:
  struct Sample
  {
    rclcpp::Time stamp;
    int traj_id{0};
    double t_closest{0.0};
    double t_progress{0.0};
    double t_cur{0.0};
    double t_duration{0.0};
    Eigen::Vector3d odom_pos{Eigen::Vector3d::Zero()};
    double odom_yaw{0.0};
    Eigen::Vector3d cmd_pos{Eigen::Vector3d::Zero()};
    double cmd_yaw{0.0};
    Eigen::Vector3d closest_pos{Eigen::Vector3d::Zero()};
    double track_yaw{0.0};
    Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
    double dist_closest_xy{0.0};
    double dist_cmd_xy{0.0};
    double dist_end_xy{0.0};
    double e_lat{0.0};
    double h_err{0.0};
  };

  void configure(const std::string & path, int period_ms)
  {
    std::lock_guard<std::mutex> lock(mu_);
    closeUnlocked();
    period_ms_ = std::max(period_ms, 1);
    if (path.empty())
      return;

    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_.is_open())
    {
      if (g_node)
      {
        RCLCPP_ERROR(
          g_node->get_logger(), "Failed to open tracking_trace CSV: %s", path.c_str());
      }
      return;
    }

    path_ = path;
    out_ << "stamp_sec,stamp_nsec,traj_id,"
         << "t_closest,t_progress,t_cur,t_duration,"
         << "odom_x,odom_y,odom_z,odom_yaw,"
         << "cmd_x,cmd_y,cmd_z,cmd_yaw,"
         << "closest_x,closest_y,closest_z,track_yaw,"
         << "vel_x,vel_y,vel_z,vel_norm,"
         << "dist_closest_xy,dist_cmd_xy,dist_end_xy,"
         << "e_lat,h_err\n";
    out_.flush();
    last_write_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (g_node)
    {
      RCLCPP_INFO(
        g_node->get_logger(), "tracking_trace CSV: %s (period_ms=%d)",
        path_.c_str(), period_ms_);
    }
  }

  void maybeWrite(const Sample & sample)
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (!out_.is_open())
      return;

    if (last_write_.nanoseconds() > 0)
    {
      const double dt_ms = (sample.stamp - last_write_).seconds() * 1000.0;
      if (dt_ms + 1e-6 < period_ms_)
        return;
    }

    const double vel_norm = sample.vel.head<2>().norm();

    out_ << sample.stamp.seconds() << ','
         << sample.stamp.nanoseconds() << ','
         << sample.traj_id << ','
         << sample.t_closest << ','
         << sample.t_progress << ','
         << sample.t_cur << ','
         << sample.t_duration << ','
         << sample.odom_pos(0) << ','
         << sample.odom_pos(1) << ','
         << sample.odom_pos(2) << ','
         << sample.odom_yaw << ','
         << sample.cmd_pos(0) << ','
         << sample.cmd_pos(1) << ','
         << sample.cmd_pos(2) << ','
         << sample.cmd_yaw << ','
         << sample.closest_pos(0) << ','
         << sample.closest_pos(1) << ','
         << sample.closest_pos(2) << ','
         << sample.track_yaw << ','
         << sample.vel(0) << ','
         << sample.vel(1) << ','
         << sample.vel(2) << ','
         << vel_norm << ','
         << sample.dist_closest_xy << ','
         << sample.dist_cmd_xy << ','
         << sample.dist_end_xy << ','
         << sample.e_lat << ','
         << sample.h_err << '\n';
    out_.flush();
    last_write_ = sample.stamp;
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mu_);
    closeUnlocked();
  }

private:
  void closeUnlocked()
  {
    if (out_.is_open())
      out_.close();
    path_.clear();
    last_write_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  std::mutex mu_;
  std::ofstream out_;
  std::string path_;
  int period_ms_{50};
  rclcpp::Time last_write_{0, 0, RCL_ROS_TIME};
};

TrackingTraceCsv g_tracking_trace_csv;

double yawFromQuaternion(double w, double x, double y, double z)
{
  const double n2 = w * w + x * x + y * y + z * z;
  if (!std::isfinite(n2) || n2 < 1e-8)
    return 0.0;
  const double inv_n = 1.0 / std::sqrt(n2);
  w *= inv_n;
  x *= inv_n;
  y *= inv_n;
  z *= inv_n;
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrapPi(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double signedLateralError(
  double robot_x, double robot_y, double ref_x, double ref_y, double path_yaw)
{
  const double ex = robot_x - ref_x;
  const double ey = robot_y - ref_y;
  const double ux = std::cos(path_yaw);
  const double uy = std::sin(path_yaw);
  return uy * ex - ux * ey;
}

constexpr double kClosestSearchDt = 0.05;
constexpr double kClosestSearchWindowSec = 1.0;

/// Closest trajectory parameter t (XY) to query. t_hint < 0 => full scan (new traj).
/// Otherwise search [t_hint ± window]; if the minimum lies on a window edge, fall back to full scan.
double closestTimeOnTrajXY(const Eigen::Vector2d & query_xy, double t_hint)
{
  if (!receive_traj_ || traj_duration_ < 1e-6)
    return 0.0;

  double best_t = 0.0;
  double best_d2 = 1e18;

  const auto scan_range = [&](const double t0, const double t1) {
    const double lo = std::max(0.0, t0);
    const double hi = std::min(traj_duration_, t1);
    for (double t = lo; t <= hi + 1e-6; t += kClosestSearchDt)
    {
      const Eigen::Vector3d p = traj_[0].evaluateDeBoorT(t);
      const double d2 = (p.head<2>() - query_xy).squaredNorm();
      if (d2 < best_d2)
      {
        best_d2 = d2;
        best_t = t;
      }
    }
  };

  if (!std::isfinite(t_hint) || t_hint < 0.0)
  {
    scan_range(0.0, traj_duration_);
    return best_t;
  }

  const double t_hint_clamped = std::max(0.0, std::min(t_hint, traj_duration_));
  const double t_lo = t_hint_clamped - kClosestSearchWindowSec;
  const double t_hi = t_hint_clamped + kClosestSearchWindowSec;
  scan_range(t_lo, t_hi);

  const double lo = std::max(0.0, t_lo);
  const double hi = std::min(traj_duration_, t_hi);
  const bool at_left_edge = lo > 1e-6 && best_t <= lo + kClosestSearchDt + 1e-6;
  const bool at_right_edge = hi < traj_duration_ - 1e-6 && best_t >= hi - kClosestSearchDt - 1e-6;
  if (at_left_edge || at_right_edge)
  {
    best_d2 = 1e18;
    scan_range(0.0, traj_duration_);
  }

  return best_t;
}
}  // namespace

double tangentYawAtTrajTime(double t)
{
  t = std::max(0.0, std::min(t, traj_duration_));
  const Eigen::Vector3d v = traj_[1].evaluateDeBoorT(t);
  if (v.head<2>().norm() > 1e-3) {
    return std::atan2(v(1), v(0));
  }
  const double tf = std::min(t + time_forward_, traj_duration_);
  const Eigen::Vector3d p0 = traj_[0].evaluateDeBoorT(t);
  const Eigen::Vector3d dir = traj_[0].evaluateDeBoorT(tf) - p0;
  if (dir.head<2>().norm() > 0.05) {
    return std::atan2(dir(1), dir(0));
  }
  return std::isfinite(last_yaw_) ? last_yaw_ : 0.0;
}

void publishExecBsplinePath(const rclcpp::Time & stamp)
{
  if (!exec_bspline_path_pub || !receive_traj_)
    return;

  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = "global";

  constexpr double dt = 0.05;
  for (double t = 0.0; t <= traj_duration_ + 1e-6; t += dt)
  {
    const Eigen::Vector3d p = traj_[0].evaluateDeBoorT(t);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p(0);
    ps.pose.position.y = p(1);
    ps.pose.position.z = p(2);
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }

  exec_bspline_path_pub->publish(path);
}

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  t_progress_ = 0.0;
  if (use_odom_progress_ && have_odom_) {
    t_progress_ = closestTimeOnTrajXY(odom_pos_.head<2>(), -1.0);
    t_progress_ = std::max(0.0, std::min(t_progress_, traj_duration_));
  }

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;
  if (have_odom_ && traj_duration_ > 1e-6) {
    const double t_yaw = std::min(t_progress_ + time_forward_, traj_duration_);
    const Eigen::Vector3d v_ref = traj_[1].evaluateDeBoorT(t_yaw);
    if (v_ref.head<2>().norm() > 0.05) {
      last_yaw_ = std::atan2(v_ref(1), v_ref(0));
    } else {
      const Eigen::Vector3d p_ref = traj_[0].evaluateDeBoorT(t_yaw);
      const Eigen::Vector2d diff = (p_ref.head<2>() - odom_pos_.head<2>());
      if (diff.norm() > 0.05) {
        last_yaw_ = std::atan2(diff.y(), diff.x());
      }
    }
  }

  receive_traj_ = true;
  publishExecBsplinePath(rclcpp::Time(msg->start_time.sec, msg->start_time.nanosec));

  if (g_node) {
    std::vector<geometry_msgs::msg::Point> pts(msg->pos_pts.begin(), msg->pos_pts.end());
    const Eigen::Vector3d traj_end = traj_[0].evaluateDeBoorT(traj_duration_);
    RCLCPP_INFO(
      g_node->get_logger(),
      "[bspline_rx] traj_id=%d dur=%.2fs odom=%s traj_end=%s %s",
      traj_id_, traj_duration_,
      traj_utils::formatVec3(odom_pos_).c_str(),
      traj_utils::formatVec3(traj_end).c_str(),
      traj_utils::formatControlPointsSummary(pts).c_str());
  }
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
  const auto & q = msg->pose.pose.orientation;
  odom_yaw_ = yawFromQuaternion(q.w, q.x, q.y, q.z);
  have_odom_ = true;
}

void cmdCallback()
{
  if (!receive_traj_)
    return;

  rclcpp::Time time_now = g_node->now();
  const Eigen::Vector3d traj_end = traj_[0].evaluateDeBoorT(traj_duration_);

  double t_closest = 0.0;
  double t_cur = 0.0;
  if (use_odom_progress_ && have_odom_)
  {
    t_closest = closestTimeOnTrajXY(odom_pos_.head<2>(), t_progress_);
    if (t_closest >= traj_duration_ - 1e-3)
      t_progress_ = t_closest;
    else
      t_progress_ = std::max(t_progress_, t_closest);
    t_cur = std::min(t_progress_ + odom_lookahead_time_, traj_duration_);
  }
  else
  {
    t_cur = (time_now - start_time_).seconds();
  }
  t_cur = std::max(0.0, std::min(t_cur, traj_duration_));

  Eigen::Vector3d pos = traj_[0].evaluateDeBoorT(t_cur);
  Eigen::Vector3d vel = traj_[1].evaluateDeBoorT(t_cur);
  Eigen::Vector3d acc = traj_[2].evaluateDeBoorT(t_cur);

  const double dist_odom_end_xy = have_odom_
    ? (odom_pos_.head<2>() - traj_end.head<2>()).norm()
    : (pos.head<2>() - traj_end.head<2>()).norm();

  if (dist_odom_end_xy <= endpoint_stop_dist_)
  {
    pos = traj_end;
    vel.setZero();
    acc.setZero();
  }

  static rclcpp::Time time_last = time_now;
  const double dt = std::max((time_now - time_last).seconds(), 1e-3);
  time_last = time_now;

  double yaw = last_yaw_;
  double yawdot = 0.0;
  const double vxy = vel.head<2>().norm();
  if (vxy > 0.05)
  {
    const double yaw_vel = std::atan2(vel(1), vel(0));
    yaw = yaw_vel;
    if (std::isfinite(last_yaw_))
      yawdot = wrapPi(yaw_vel - last_yaw_) / dt;
    yawdot = std::clamp(yawdot, -max_yaw_dot_, max_yaw_dot_);
  }
  if (!std::isfinite(yaw))
    yaw = std::isfinite(last_yaw_) ? last_yaw_ : 0.0;
  if (!std::isfinite(yawdot))
    yawdot = 0.0;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "global";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  if (!std::isfinite(cmd.velocity.x)) cmd.velocity.x = 0.0;
  if (!std::isfinite(cmd.velocity.y)) cmd.velocity.y = 0.0;
  if (!std::isfinite(cmd.velocity.z)) cmd.velocity.z = 0.0;
  if (!std::isfinite(cmd.yaw)) cmd.yaw = 0.0;

  double t_track = t_closest;
  if (!use_odom_progress_ || !have_odom_)
    t_track = t_cur;
  t_track = std::max(0.0, std::min(t_track, traj_duration_));
  const Eigen::Vector3d track_pos = traj_[0].evaluateDeBoorT(t_track);
  cmd.track_point.x = track_pos(0);
  cmd.track_point.y = track_pos(1);
  cmd.track_point.z = track_pos(2);
  cmd.track_yaw = tangentYawAtTrajTime(t_track);
  if (!std::isfinite(cmd.track_yaw))
    cmd.track_yaw = cmd.yaw;

  pos_cmd_pub->publish(cmd);

  if (g_node && have_odom_) {
    const double dist_odom_cmd = (odom_pos_.head<2>() - pos.head<2>()).norm();
    RCLCPP_INFO_THROTTLE(
      g_node->get_logger(), *g_node->get_clock(),
      std::max(g_log_trace_period_ms_, 1),
      "[pos_cmd_pub] traj_id=%d t=%.2f/%.2f odom=%s cmd_pos=%s vel=%s dist_odom_cmd=%.3f",
      traj_id_, t_cur, traj_duration_,
      traj_utils::formatVec3(odom_pos_).c_str(),
      traj_utils::formatVec3(pos).c_str(),
      traj_utils::formatXYZ(vel(0), vel(1), vel(2)).c_str(),
      dist_odom_cmd);

    const double t_closest_log = closestTimeOnTrajXY(
      odom_pos_.head<2>(), use_odom_progress_ ? t_progress_ : -1.0);
    const Eigen::Vector3d closest_pos_log = traj_[0].evaluateDeBoorT(t_closest_log);
    const double track_yaw_log = tangentYawAtTrajTime(t_closest_log);
    const double dist_closest_xy =
      (odom_pos_.head<2>() - closest_pos_log.head<2>()).norm();
    const double e_lat = signedLateralError(
      odom_pos_(0), odom_pos_(1),
      closest_pos_log(0), closest_pos_log(1), track_yaw_log);
    const double h_err = wrapPi(track_yaw_log - odom_yaw_);

    TrackingTraceCsv::Sample trace;
    trace.stamp = time_now;
    trace.traj_id = traj_id_;
    trace.t_closest = t_closest_log;
    trace.t_progress = t_progress_;
    trace.t_cur = t_cur;
    trace.t_duration = traj_duration_;
    trace.odom_pos = odom_pos_;
    trace.odom_yaw = odom_yaw_;
    trace.cmd_pos = pos;
    trace.cmd_yaw = cmd.yaw;
    trace.closest_pos = closest_pos_log;
    trace.track_yaw = track_yaw_log;
    trace.vel = vel;
    trace.dist_closest_xy = dist_closest_xy;
    trace.dist_cmd_xy = dist_odom_cmd;
    trace.dist_end_xy = dist_odom_end_xy;
    trace.e_lat = e_lat;
    trace.h_err = h_err;
    g_tracking_trace_csv.maybeWrite(trace);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");
  g_node = node;

  node->declare_parameter("traj_server/time_forward", 1.0);
  node->declare_parameter("traj_server/use_odom_progress", false);
  node->declare_parameter("traj_server/odom_lookahead_time", 0.4);
  node->declare_parameter("traj_server/log_trace_period_ms", 500);
  node->declare_parameter("traj_server/tracking_trace_csv", std::string(""));
  node->declare_parameter("traj_server/tracking_trace_period_ms", 50);
  node->declare_parameter("traj_server/endpoint_stop_dist", 0.3);
  node->declare_parameter("traj_server/max_yaw_dot", 0.5);
  node->get_parameter("traj_server/time_forward", time_forward_);
  node->get_parameter("traj_server/use_odom_progress", use_odom_progress_);
  node->get_parameter("traj_server/odom_lookahead_time", odom_lookahead_time_);
  node->get_parameter("traj_server/log_trace_period_ms", g_log_trace_period_ms_);
  std::string tracking_trace_csv;
  int tracking_trace_period_ms = 50;
  node->get_parameter("traj_server/tracking_trace_csv", tracking_trace_csv);
  node->get_parameter("traj_server/tracking_trace_period_ms", tracking_trace_period_ms);
  g_tracking_trace_csv.configure(tracking_trace_csv, tracking_trace_period_ms);
  node->get_parameter("traj_server/endpoint_stop_dist", endpoint_stop_dist_);
  node->get_parameter("traj_server/max_yaw_dot", max_yaw_dot_);

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  // High-rate VIO odom: best_effort + keep_last(1) (latest-sample-wins, no backlog).
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), odomCallback);

  // Relative name so launch remapping (position_cmd -> /drone_0_planning/pos_cmd) applies.
  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "position_cmd",
      50);

  exec_bspline_path_pub = node->create_publisher<nav_msgs::msg::Path>(
      "planning/exec_bspline_path",
      1);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  if (use_odom_progress_)
    RCLCPP_WARN(node->get_logger(), "[Traj server]: ready (odom-synced playback).");
  else
    RCLCPP_WARN(node->get_logger(), "[Traj server]: ready (wall-clock playback).");

  rclcpp::on_shutdown([]() { g_tracking_trace_csv.close(); });

  rclcpp::spin(node);
  g_tracking_trace_csv.close();
  rclcpp::shutdown();

  return 0;
}
