#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

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
double cmd_speed_max_ = 0.6;
double min_vel_fill_dist_ = 0.2;
double min_vel_fill_threshold_ = 0.08;

/// If spline vel≈0 but robot is still far from commanded pos, emit world-frame XY velocity toward pos.
void ensureVelocityTowardCommand(Eigen::Vector3d & vel, const Eigen::Vector3d & pos)
{
  if (!use_odom_progress_ || !have_odom_)
    return;

  const double dist_xy = (odom_pos_.head<2>() - pos.head<2>()).norm();
  if (dist_xy < min_vel_fill_dist_ || vel.head<2>().norm() > min_vel_fill_threshold_)
    return;

  Eigen::Vector2d dir = pos.head<2>() - odom_pos_.head<2>();
  const double n = dir.norm();
  if (n < 1e-3)
    return;
  dir /= n;

  const double speed = std::min(cmd_speed_max_, std::max(0.15, 0.5 * dist_xy));
  vel(0) = dir(0) * speed;
  vel(1) = dir(1) * speed;
  vel(2) = 0.0;
}

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
  if (use_odom_progress_ && have_odom_)
    t_progress_ = closestTimeOnTrajXY(odom_pos_.head<2>());

  receive_traj_ = true;
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
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw;
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
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

  rclcpp::Clock clock(RCL_ROS_TIME);
  rclcpp::Time time_now = clock.now();

  double t_cur = 0.0;
  if (use_odom_progress_ && have_odom_)
  {
    const double t_closest = closestTimeOnTrajXY(odom_pos_.head<2>());
    t_progress_ = std::max(t_progress_, t_closest);
    t_cur = std::min(t_progress_ + odom_lookahead_time_, traj_duration_);

    // Slow ground robot: don't command vel=0 at traj end while odom is still far from endpoint.
    if (t_cur >= traj_duration_ - 1e-3)
    {
      const Eigen::Vector3d end_pos = traj_[0].evaluateDeBoorT(traj_duration_);
      if ((odom_pos_.head<2>() - end_pos.head<2>()).norm() > 0.35)
        t_cur = std::min(t_closest + odom_lookahead_time_, traj_duration_ - 1e-3);
    }
  }
  else
  {
    t_cur = (time_now - start_time_).seconds();
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
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
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel = traj_[1].evaluateDeBoorT(std::max(traj_duration_ - 1e-3, 0.0));
    acc.setZero();
    ensureVelocityTowardCommand(vel, pos);

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }

  ensureVelocityTowardCommand(vel, pos);

  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
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

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  node->declare_parameter("traj_server/time_forward", 1.0);
  node->declare_parameter("traj_server/use_odom_progress", false);
  node->declare_parameter("traj_server/odom_lookahead_time", 0.4);
  node->declare_parameter("traj_server/cmd_speed_max", 0.6);
  node->declare_parameter("traj_server/min_vel_fill_dist", 0.2);
  node->get_parameter("traj_server/time_forward", time_forward_);
  node->get_parameter("traj_server/use_odom_progress", use_odom_progress_);
  node->get_parameter("traj_server/odom_lookahead_time", odom_lookahead_time_);
  node->get_parameter("traj_server/cmd_speed_max", cmd_speed_max_);
  node->get_parameter("traj_server/min_vel_fill_dist", min_vel_fill_dist_);

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
