#include "d1_planner_bridge/d1_planner_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace d1_planner_bridge
{

D1PlannerBridgeNode::D1PlannerBridgeNode(const rclcpp::NodeOptions & options)
: Node("d1_planner_bridge_node", options)
{
  const TrackerParams tracker_params = loadTrackerParams();
  need_odom_ = tracker_params.project_velocity_to_body;
  tracker_.setParams(tracker_params);

  pos_cmd_topic_ = declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/ov_msckf/odomimu");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/command/cmd_twist");

  const double control_rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
  log_cmd_vel_ = declare_parameter<bool>("log_cmd_vel", true);
  log_cmd_vel_period_ms_ = declare_parameter<int>("log_cmd_vel_period_ms", 500);
  cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.3);
  odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);

  pos_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
    pos_cmd_topic_, rclcpp::QoS(10),
    std::bind(&D1PlannerBridgeNode::posCmdCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&D1PlannerBridgeNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0));
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&D1PlannerBridgeNode::controlTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "d1_planner_bridge: body vx + heading P + yaw_dot -> %s",
    cmd_vel_topic_.c_str());
  if (log_cmd_vel_) {
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel logging enabled (period_ms=%d, 0 = every publish)",
      log_cmd_vel_period_ms_);
  }
}

TrackerParams D1PlannerBridgeNode::loadTrackerParams()
{
  TrackerParams p;
  p.max_vx = declare_parameter<double>("max_vx", 0.6);
  p.max_wz = declare_parameter<double>("max_wz", 0.5);
  p.yaw_kp = declare_parameter<double>("yaw_kp", 1.2);
  p.yaw_rate_ff = declare_parameter<double>("yaw_rate_ff", 1.0);
  p.align_heading_thresh_rad = declare_parameter<double>("align_heading_thresh_rad", 0.4);
  p.min_turn_wz = declare_parameter<double>("min_turn_wz", 0.15);
  p.project_velocity_to_body = declare_parameter<bool>("project_velocity_to_body", true);
  p.allow_reverse = declare_parameter<bool>("allow_reverse", false);
  return p;
}

void D1PlannerBridgeNode::posCmdCallback(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pos_cmd_ = msg;
  last_pos_cmd_recv_ = now();
  have_pos_cmd_recv_ = true;
}

void D1PlannerBridgeNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = msg;
  last_odom_recv_ = now();
  have_odom_recv_ = true;
}

void D1PlannerBridgeNode::controlTimerCallback()
{
  quadrotor_msgs::msg::PositionCommand::SharedPtr cmd;
  nav_msgs::msg::Odometry::SharedPtr odom;
  rclcpp::Time cmd_recv, odom_recv;
  bool have_cmd_recv = false, have_odom_recv = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = last_pos_cmd_;
    odom = last_odom_;
    cmd_recv = last_pos_cmd_recv_;
    odom_recv = last_odom_recv_;
    have_cmd_recv = have_pos_cmd_recv_;
    have_odom_recv = have_odom_recv_;
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;

  if (!cmd) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No pos_cmd on '%s' yet", pos_cmd_topic_.c_str());
    cmd_vel_pub_->publish(twist);
    return;
  }

  const rclcpp::Time t_now = now();
  const double inf = std::numeric_limits<double>::infinity();
  const double cmd_age = have_cmd_recv ? (t_now - cmd_recv).seconds() : inf;
  const double odom_age = have_odom_recv ? (t_now - odom_recv).seconds() : inf;
  const bool cmd_stale = cmd_age > cmd_timeout_sec_;
  const bool odom_stale = need_odom_ && (odom_age > odom_timeout_sec_);

  if (cmd_stale || odom_stale) {
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[watchdog] stale input -> stop. cmd_age=%.3fs (timeout=%.2f) "
      "odom_age=%.3fs (timeout=%.2f need_odom=%d)",
      cmd_age, cmd_timeout_sec_, odom_age, odom_timeout_sec_, need_odom_ ? 1 : 0);
    return;
  }

  const GroundTwist ground = tracker_.compute(*cmd, odom ? odom.get() : nullptr);
  if (ground.valid) {
    twist.linear.x = std::isfinite(ground.vx) ? ground.vx : 0.0;
    twist.angular.z = std::isfinite(ground.wz) ? ground.wz : 0.0;
  }

  cmd_vel_pub_->publish(twist);

  if (log_cmd_vel_) {
    const int throttle_ms = std::max(log_cmd_vel_period_ms_, 0);
    if (ground.valid) {
      double odom_x = 0.0, odom_y = 0.0, odom_yaw = 0.0;
      if (odom) {
        odom_x = odom->pose.pose.position.x;
        odom_y = odom->pose.pose.position.y;
        odom_yaw = TrajectoryTracker::forwardYawFromOdom(*odom);
      }
      const double dist_odom_cmd = odom ?
        std::hypot(
          odom_x - cmd->position.x,
          odom_y - cmd->position.y) :
        -1.0;
      const double plan_vel = std::hypot(cmd->velocity.x, cmd->velocity.y);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), throttle_ms,
        "[cmd_vel_pub] odom=(%.3f,%.3f,yaw=%.3f) cmd_pos=(%.3f,%.3f) "
        "cmd_vel=(%.3f,%.3f) cmd_yaw_dot=%.3f twist=(%.3f,%.3f) "
        "plan_vel=%.3f h_err=%.3f dist=%.3f traj_id=%u",
        odom_x, odom_y, odom_yaw,
        cmd->position.x, cmd->position.y,
        cmd->velocity.x, cmd->velocity.y,
        std::isfinite(cmd->yaw_dot) ? cmd->yaw_dot : 0.0,
        twist.linear.x, twist.angular.z,
        plan_vel, ground.heading_error, dist_odom_cmd,
        cmd->trajectory_id);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), throttle_ms,
        "D1 %s: linear.x=%.4f angular.z=%.4f (trajectory not READY, flag=%u)",
        cmd_vel_topic_.c_str(),
        twist.linear.x, twist.angular.z,
        cmd->trajectory_flag);
    }
  }
}

}  // namespace d1_planner_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<d1_planner_bridge::D1PlannerBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
