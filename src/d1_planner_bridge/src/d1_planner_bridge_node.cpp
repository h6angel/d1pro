#include "d1_planner_bridge/d1_planner_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace d1_planner_bridge
{

D1PlannerBridgeNode::D1PlannerBridgeNode(const rclcpp::NodeOptions & options)
: Node("d1_planner_bridge_node", options)
{
  tracker_.setParams(loadTrackerParams());

  pos_cmd_topic_ = declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/ov_msckf/odomimu");
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
    "d1_planner_bridge: pos_cmd vel feedforward + yaw/lateral feedback -> %s",
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
  p.yaw_kp = declare_parameter<double>("yaw_kp", 0.8);
  p.yaw_rate_ff = declare_parameter<double>("yaw_rate_ff", 1.0);
  p.max_yaw_dot_ff = declare_parameter<double>("max_yaw_dot_ff", 0.5);
  p.project_velocity_to_body = declare_parameter<bool>("project_velocity_to_body", true);
  p.min_vx = declare_parameter<double>("min_vx", 0.08);
  p.vx_deadband = declare_parameter<double>("vx_deadband", 0.01);
  p.max_wz_yaw_p = declare_parameter<double>("max_wz_yaw_p", 0.5);
  p.min_turn_wz = declare_parameter<double>("min_turn_wz", 0.5);
  p.align_heading_thresh_rad = declare_parameter<double>("align_heading_thresh_rad", M_PI / 3.0);
  p.allow_reverse = declare_parameter<bool>("allow_reverse", false);
  p.enable_lateral_correction = declare_parameter<bool>("enable_lateral_correction", true);
  p.lateral_kp = declare_parameter<double>("lateral_kp", 0.3);
  p.lateral_error_deadband = declare_parameter<double>("lateral_error_deadband", 0.05);
  p.max_wz_lateral_p = declare_parameter<double>("max_wz_lateral_p", 0.35);
  p.vx_lat_damp_gain = declare_parameter<double>("vx_lat_damp_gain", 0.4);
  p.lateral_slowdown_dist = declare_parameter<double>("lateral_slowdown_dist", 0.4);
  p.min_plan_speed_for_lateral = declare_parameter<double>("min_plan_speed_for_lateral", 0.15);
  p.max_lateral_error_m = declare_parameter<double>("max_lateral_error_m", 2.0);
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
      double odom_x = 0.0, odom_y = 0.0, odom_yaw = 0.0;
      if (odom) {
        odom_x = odom->pose.pose.position.x;
        odom_y = odom->pose.pose.position.y;
        const auto & q = odom->pose.pose.orientation;
        odom_yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      }
      const double dist_odom_cmd = odom ?
        std::hypot(
          odom_x - cmd->position.x,
          odom_y - cmd->position.y) :
        -1.0;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), throttle_ms,
        "[cmd_vel_pub] odom=(%.3f,%.3f,yaw=%.3f) cmd_pos=(%.3f,%.3f) cmd_vel=(%.3f,%.3f) "
        "twist=(%.3f,%.3f) dist=%.3f e_lat=%.3f h_err=%.3f traj_id=%u",
        odom_x, odom_y, odom_yaw,
        cmd->position.x, cmd->position.y,
        cmd->velocity.x, cmd->velocity.y,
        twist.linear.x, twist.angular.z,
        dist_odom_cmd, ground.lateral_error, ground.heading_error,
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
