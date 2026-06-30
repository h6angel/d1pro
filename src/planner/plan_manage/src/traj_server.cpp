#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/trajectory_debug_log.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
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

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

// Odom-synced playback (for slow ground robots / D1)
bool use_odom_progress_ = false;
bool have_odom_ = false;
Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
double t_progress_ = 0.0;
double odom_lookahead_time_ = 0.4;
// When odom is past traj end but not at endpoint: guide toward traj_end instead of freezing bad vel.
double endpoint_approach_dist_ = 0.35;
double endpoint_stop_dist_ = 0.08;
double endpoint_vel_gain_ = 0.8;
double endpoint_max_vel_ = 0.6;
double max_yaw_dot_ = 0.5;

rclcpp::Node::SharedPtr g_node;
int g_log_trace_period_ms_ = 500;

double closestTimeOnTrajXY(const Eigen::Vector2d & query_xy)
{
  double best_t = 0.0;
  double best_d2 = 1e18;
  constexpr double dt = 0.05;
  for (double t = 0.0; t <= traj_duration_ + 1e-6; t += dt)
  {
    const Eigen::Vector3d p = traj_[0].evaluateDeBoorT(t);
    const double d2 = (p.head<2>() - query_xy).squaredNorm();
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best_t = t;
    }
  }
  return best_t;
}

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
    t_progress_ = closestTimeOnTrajXY(odom_pos_.head<2>());
    t_progress_ = std::max(0.0, std::min(t_progress_, traj_duration_));
  }

  last_yaw_dot_ = 0.0;
  if (!std::isfinite(last_yaw_))
    last_yaw_ = 0.0;
  if (have_odom_ && traj_duration_ > 1e-6) {
    const double t_yaw = std::min(t_progress_ + time_forward_, traj_duration_);
    const Eigen::Vector3d p_ref = traj_[0].evaluateDeBoorT(t_yaw);
    const Eigen::Vector2d diff = (p_ref.head<2>() - odom_pos_.head<2>());
    if (diff.norm() > 0.05)
      last_yaw_ = std::atan2(diff.y(), diff.x());
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
  have_odom_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  const double yaw_dot_max = std::max(max_yaw_dot_, 1e-3);
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  if (!std::isfinite(last_yaw_))
    last_yaw_ = 0.0;
  if (!std::isfinite(last_yaw_dot_))
    last_yaw_dot_ = 0.0;

  const double dt = std::max((time_now - time_last).seconds(), 1e-3);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  if (!std::isfinite(yaw_temp))
    yaw_temp = last_yaw_;
  double max_yaw_change = yaw_dot_max * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -yaw_dot_max;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -yaw_dot_max;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = yaw_dot_max;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = yaw_dot_max;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -yaw_dot_max;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = yaw_dot_max;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -yaw_dot_max;
      else if (yaw - last_yaw_ < -PI)
        yawdot = yaw_dot_max;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw;
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;

  if (!std::isfinite(yaw))
    yaw = last_yaw_;
  if (!std::isfinite(yawdot))
    yawdot = 0.0;

  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  if (!receive_traj_)
    return;

  // Use the node ROS clock so wall-clock playback matches start_time_ (set with the
  // planner node clock) and respects use_sim_time. A standalone rclcpp::Clock is not
  // driven by /clock and would diverge under simulation.
  rclcpp::Time time_now = g_node->now();

  const Eigen::Vector3d traj_end = traj_[0].evaluateDeBoorT(traj_duration_);

  double t_cur = 0.0;
  bool endpoint_hold = false;
  if (use_odom_progress_ && have_odom_)
  {
    const double t_closest = closestTimeOnTrajXY(odom_pos_.head<2>());
    const double dist_odom_end_xy =
      (odom_pos_.head<2>() - traj_end.head<2>()).norm();

    // Overshoot past traj end: allow t_progress to move backward with closest projection.
    if (t_closest >= traj_duration_ - 1e-3 &&
        dist_odom_end_xy > endpoint_approach_dist_)
    {
      t_progress_ = t_closest;
    }
    else
    {
      t_progress_ = std::max(t_progress_, t_closest);
    }
    t_cur = std::min(t_progress_ + odom_lookahead_time_, traj_duration_);

    if (t_cur >= traj_duration_ - 1e-3 &&
        dist_odom_end_xy > endpoint_stop_dist_)
    {
      endpoint_hold = true;
    }
  }
  else
  {
    t_cur = (time_now - start_time_).seconds();
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = time_now;
  if (endpoint_hold)
  {
    pos = traj_end;
    acc.setZero();
    pos_f = traj_end;

    const Eigen::Vector2d diff =
      traj_end.head<2>() - odom_pos_.head<2>();
    const double dist_xy = diff.norm();
    vel.setZero();
    if (dist_xy > endpoint_stop_dist_)
    {
      const double spd = std::min(endpoint_max_vel_, endpoint_vel_gain_ * dist_xy);
      vel.head<2>() = diff / dist_xy * spd;
    }

    if (vel.head<2>().norm() > 0.05)
    {
      yaw_yawdot.first = std::atan2(vel(1), vel(0));
      yaw_yawdot.second = 0.0;
    }
    else
    {
      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0.0;
    }
  }
  else if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    pos = traj_end;
    acc.setZero();
    pos_f = traj_end;

    const double dist_odom_end_xy = have_odom_ ?
      (odom_pos_.head<2>() - traj_end.head<2>()).norm() : 0.0;
    if (have_odom_ && dist_odom_end_xy <= endpoint_stop_dist_)
    {
      vel.setZero();
      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0.0;
    }
    else
    {
      vel = traj_[1].evaluateDeBoorT(std::max(traj_duration_ - 1e-3, 0.0));
      yaw_yawdot = calculate_yaw(
        std::max(traj_duration_ - 1e-3, 0.0), pos, time_now, time_last);
    }
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }

  time_last = time_now;

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

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  if (!std::isfinite(cmd.velocity.x)) cmd.velocity.x = 0.0;
  if (!std::isfinite(cmd.velocity.y)) cmd.velocity.y = 0.0;
  if (!std::isfinite(cmd.velocity.z)) cmd.velocity.z = 0.0;
  if (!std::isfinite(cmd.yaw)) cmd.yaw = std::isfinite(last_yaw_) ? last_yaw_ : 0.0;
  if (!std::isfinite(cmd.yaw_dot)) cmd.yaw_dot = 0.0;

  double t_track = std::max(0.0, std::min(t_cur, traj_duration_));
  if (use_odom_progress_ && have_odom_) {
    t_track = closestTimeOnTrajXY(odom_pos_.head<2>());
  }
  const Eigen::Vector3d track_pos = traj_[0].evaluateDeBoorT(t_track);
  cmd.track_point.x = track_pos(0);
  cmd.track_point.y = track_pos(1);
  cmd.track_point.z = track_pos(2);
  cmd.track_yaw = tangentYawAtTrajTime(t_track);
  if (!std::isfinite(cmd.track_yaw)) {
    cmd.track_yaw = cmd.yaw;
  }

  last_yaw_ = cmd.yaw;

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
  node->declare_parameter("traj_server/endpoint_approach_dist", 0.35);
  node->declare_parameter("traj_server/endpoint_stop_dist", 0.08);
  node->declare_parameter("traj_server/endpoint_vel_gain", 0.8);
  node->declare_parameter("traj_server/endpoint_max_vel", 0.6);
  node->declare_parameter("traj_server/max_yaw_dot", 0.5);
  node->get_parameter("traj_server/time_forward", time_forward_);
  node->get_parameter("traj_server/use_odom_progress", use_odom_progress_);
  node->get_parameter("traj_server/odom_lookahead_time", odom_lookahead_time_);
  node->get_parameter("traj_server/log_trace_period_ms", g_log_trace_period_ms_);
  node->get_parameter("traj_server/endpoint_approach_dist", endpoint_approach_dist_);
  node->get_parameter("traj_server/endpoint_stop_dist", endpoint_stop_dist_);
  node->get_parameter("traj_server/endpoint_vel_gain", endpoint_vel_gain_);
  node->get_parameter("traj_server/endpoint_max_vel", endpoint_max_vel_);
  node->get_parameter("traj_server/max_yaw_dot", max_yaw_dot_);

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odomCallback);

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

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
