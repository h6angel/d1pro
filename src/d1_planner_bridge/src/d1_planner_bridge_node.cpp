#include "d1_planner_bridge/d1_planner_bridge.hpp"

#include <algorithm>
#include <functional>

namespace d1_planner_bridge
{

D1PlannerBridgeNode::D1PlannerBridgeNode(const rclcpp::NodeOptions & options)
: Node("d1_planner_bridge_node", options)
{
  tracker_.setParams(loadTrackerParams());

  pos_cmd_topic_ = declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/command/cmd_twist");

  const double control_rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
  log_cmd_vel_ = declare_parameter<bool>("log_cmd_vel", true);
  log_cmd_vel_period_ms_ = declare_parameter<int>("log_cmd_vel_period_ms", 500);
  cmd_vel_ema_alpha_ = declare_parameter<double>("cmd_vel_ema_alpha", 0.35);

  pos_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
    pos_cmd_topic_, rclcpp::QoS(10),
    std::bind(&D1PlannerBridgeNode::posCmdCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(10),
    std::bind(&D1PlannerBridgeNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0));
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&D1PlannerBridgeNode::controlTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "d1_planner_bridge: direct pos_cmd velocity -> %s (no stale stop / no goal stop)",
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
  p.max_wz = declare_parameter<double>("max_wz", 1.0);
  p.yaw_kp = declare_parameter<double>("yaw_kp", 1.5);
  p.yaw_rate_ff = declare_parameter<double>("yaw_rate_ff", 1.0);
  p.project_velocity_to_body = declare_parameter<bool>("project_velocity_to_body", true);
  p.min_vx = declare_parameter<double>("min_vx", 0.08);
  p.vx_deadband = declare_parameter<double>("vx_deadband", 0.01);
  p.max_wz_yaw_p = declare_parameter<double>("max_wz_yaw_p", 0.35);
  return p;
}

void D1PlannerBridgeNode::posCmdCallback(
  const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pos_cmd_ = msg;
}

void D1PlannerBridgeNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = msg;
}

void D1PlannerBridgeNode::controlTimerCallback()
{
  quadrotor_msgs::msg::PositionCommand::SharedPtr cmd;
  nav_msgs::msg::Odometry::SharedPtr odom;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = last_pos_cmd_;
    odom = last_odom_;
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

  const GroundTwist ground = tracker_.compute(*cmd, odom ? odom.get() : nullptr);
  if (ground.valid) {
    const double a = std::clamp(cmd_vel_ema_alpha_, 0.0, 1.0);
    if (!cmd_vel_filter_init_) {
      filt_vx_ = ground.vx;
      filt_wz_ = ground.wz;
      cmd_vel_filter_init_ = true;
    } else if (a > 0.0) {
      filt_vx_ = a * ground.vx + (1.0 - a) * filt_vx_;
      filt_wz_ = a * ground.wz + (1.0 - a) * filt_wz_;
    } else {
      filt_vx_ = ground.vx;
      filt_wz_ = ground.wz;
    }
    twist.linear.x = filt_vx_;
    twist.angular.z = filt_wz_;
  } else {
    cmd_vel_filter_init_ = false;
    filt_vx_ = 0.0;
    filt_wz_ = 0.0;
  }

  cmd_vel_pub_->publish(twist);

  if (log_cmd_vel_) {
    const int throttle_ms = std::max(log_cmd_vel_period_ms_, 0);
    if (ground.valid) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), throttle_ms,
        "D1 %s: linear.x=%.4f angular.z=%.4f | planner vel (%.4f, %.4f, %.4f) yaw_dot=%.4f",
        cmd_vel_topic_.c_str(),
        twist.linear.x, twist.angular.z,
        cmd->velocity.x, cmd->velocity.y, cmd->velocity.z, cmd->yaw_dot);
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
