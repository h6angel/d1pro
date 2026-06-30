
#include <algorithm>
#include <limits>

#include <ego_planner/ego_replan_fsm.h>
#include "traj_utils/trajectory_debug_log.hpp"

namespace ego_planner
{

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/thresh_goal_reach_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/global_replan_drift_thresh", 0.25);
    node_->declare_parameter("fsm/odom_traj_mismatch_thresh", 0.22);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("fsm/log_trace_period_ms", 500);
    node_->declare_parameter("fsm/gen_new_traj_max_failures", 8);
    node_->declare_parameter("fsm/gen_new_traj_backoff_base_sec", 0.25);
    node_->declare_parameter("fsm/gen_new_traj_backoff_max_sec", 2.0);

    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/thresh_goal_reach_meter", goal_reach_thresh_);
    if (goal_reach_thresh_ < 0.0)
      goal_reach_thresh_ = no_replan_thresh_;
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/global_replan_drift_thresh", global_replan_drift_thresh_);
    node_->get_parameter("fsm/odom_traj_mismatch_thresh", odom_traj_mismatch_thresh_);
    odom_traj_mismatch_thresh_ = std::max(odom_traj_mismatch_thresh_, 0.01);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("fsm/log_trace_period_ms", log_trace_period_ms_);
    node_->get_parameter("fsm/gen_new_traj_max_failures", gen_new_traj_max_failures_);
    node_->get_parameter("fsm/gen_new_traj_backoff_base_sec", gen_new_traj_backoff_base_sec_);
    node_->get_parameter("fsm/gen_new_traj_backoff_max_sec", gen_new_traj_backoff_max_sec_);
    gen_new_traj_max_failures_ = std::max(gen_new_traj_max_failures_, 1);
    gen_new_traj_backoff_base_sec_ = std::max(gen_new_traj_backoff_base_sec_, 0.05);
    gen_new_traj_backoff_max_sec_ = std::max(
      gen_new_traj_backoff_max_sec_, gen_new_traj_backoff_base_sec_);

    node_->declare_parameter("fsm/enable_tag_tracking", false);
    node_->declare_parameter("fsm/tag_pose_topic", "/apriltag/target_pose_global");
    node_->declare_parameter("fsm/tag_detected_topic", "/apriltag/target_detected");
    node_->declare_parameter("fsm/tag_stop_dist", 0.25);
    node_->declare_parameter("fsm/tag_update_min_dist", 0.08);
    node_->declare_parameter("fsm/tag_replan_min_period", 0.5);

    node_->get_parameter("fsm/enable_tag_tracking", enable_tag_tracking_);
    node_->get_parameter("fsm/tag_pose_topic", tag_pose_topic_);
    node_->get_parameter("fsm/tag_detected_topic", tag_detected_topic_);
    node_->get_parameter("fsm/tag_stop_dist", tag_stop_dist_);
    tag_stop_dist_ = std::max(tag_stop_dist_, 0.05);
    node_->get_parameter("fsm/tag_update_min_dist", tag_update_min_dist_);
    node_->get_parameter("fsm/tag_replan_min_period", tag_replan_min_period_);

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);

    if (!enable_tag_tracking_)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }

    if (enable_tag_tracking_)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "AprilTag tracking enabled: pose=%s detected=%s stop_dist=%.2f m",
          tag_pose_topic_.c_str(), tag_detected_topic_.c_str(), tag_stop_dist_);

      tag_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          tag_pose_topic_, 10,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->tagPoseCallback(msg);
          });

      tag_detected_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          tag_detected_topic_, 10,
          [this](const std::shared_ptr<const std_msgs::msg::Bool> &msg)
          {
            this->tagDetectedCallback(msg);
          });
    }
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    planner_manager_->setRobotPlanningZ(odom_pos_(2));
    Eigen::Vector3d wp = next_wp;
    wp(2) = odom_pos_(2);
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM: schedule replan on next exec tick (never spin node from a callback) ***/
      resetGenNewTrajRetry();
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::resetGenNewTrajRetry()
  {
    gen_new_traj_fail_count_ = 0;
    gen_new_traj_next_attempt_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void EGOReplanFSM::onGenNewTrajPlanFailed()
  {
    gen_new_traj_fail_count_++;
    const double backoff = std::min(
      gen_new_traj_backoff_max_sec_,
      gen_new_traj_backoff_base_sec_ * static_cast<double>(gen_new_traj_fail_count_));
    gen_new_traj_next_attempt_ = node_->now() + rclcpp::Duration::from_seconds(backoff);

    if (gen_new_traj_fail_count_ >= gen_new_traj_max_failures_)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "GEN_NEW_TRAJ failed %d times (stale map / in obstacle?). "
        "Holding stop; fix depth/VIO and re-send goal.",
        gen_new_traj_fail_count_);
      resetGenNewTrajRetry();
      have_target_ = false;
      if (have_odom_)
        callEmergencyStop(odom_pos_);
      changeFSMExecState(WAIT_TARGET, "FSM");
      return;
    }

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(),
      std::max(log_trace_period_ms_, 500),
      "GEN_NEW_TRAJ plan failed (%d/%d), retry in %.2fs",
      gen_new_traj_fail_count_, gen_new_traj_max_failures_, backoff);
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (enable_tag_tracking_)
      return;

    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    const double goal_z = odom_pos_(2);

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, goal_z);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // OpenVINS odomimu: twist.linear is in child_frame (imu/body); planning uses global.
    const Eigen::Vector3d v_body(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    odom_vel_ = odom_orient_ * v_body;

    // odom_acc_ = estimateAcc( msg );

    have_odom_ = true;
    planner_manager_->updateRobotPosition(odom_pos_);
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

    if (enable_tag_tracking_)
      updateTagTrackingOnExecTick();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (gen_new_traj_next_attempt_.nanoseconds() > 0 &&
          node_->now() < gen_new_traj_next_attempt_)
      {
        goto force_return;
      }

      if (pending_estop_global_replan_)
      {
        pending_estop_global_replan_ = false;
        maybeReplanGlobalAfterEstop();
      }

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        resetGenNewTrajRetry();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        onGenNewTrajPlanFailed();
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // Warm-start from previous B-spline; pin first control points to /odom (D1).
      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        const double dist_to_goal = (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
        if (!isTagFollowing() && dist_to_goal < goal_reach_thresh_)
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      const double t_wall = (time_now - info->start_time_).seconds();
      double t_cur = std::min(info->duration_, t_wall);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      const double dist_to_goal_xy =
        (end_pt_.head<2>() - odom_pos_.head<2>()).norm();
      const bool at_goal = dist_to_goal_xy < goal_reach_thresh_;
      const double local_tgt_xy_err =
        (local_target_pt_.head<2>() - end_pt_.head<2>()).norm();
      // XY only: local_target z is overwritten with odom z in callReboundReplan().
      const bool local_target_is_goal = local_tgt_xy_err < 0.15;
      const bool near_goal_phase =
        local_target_is_goal || dist_to_goal_xy < planning_horizen_;

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (near_goal_phase)
      {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(),
          std::max(log_trace_period_ms_, 1),
          "[exec_trace] odom=%s goal=%s dist_goal_xy=%.3f t_wall=%.2f t_cur=%.2f/%.2f at_goal=%d",
          traj_utils::formatVec3(odom_pos_).c_str(),
          traj_utils::formatVec3(end_pt_).c_str(),
          dist_to_goal_xy, t_wall, t_cur, info->duration_, at_goal ? 1 : 0);

        if (t_cur > info->duration_ - 1e-2)
        {
          if (at_goal && !isTagFollowing())
          {
            RCLCPP_INFO(
              node_->get_logger(),
              "[goal_reached] odom=%s goal=%s dist_xy=%.3f -> WAIT_TARGET (stop planning)",
              traj_utils::formatVec3(odom_pos_).c_str(),
              traj_utils::formatVec3(end_pt_).c_str(),
              dist_to_goal_xy);
            have_target_ = false;

            changeFSMExecState(WAIT_TARGET, "FSM");
            goto force_return;
          }
          else
          {
            RCLCPP_WARN(
              node_->get_logger(),
              "[goal_timeout] traj ended dist_goal_xy=%.3f > thresh=%.3f -> REPLAN",
              dist_to_goal_xy, goal_reach_thresh_);
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
        else if (!at_goal && t_wall > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_wall > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (odom_vel_.norm() < 0.1)
      {
        if (pending_tag_estop_to_wait_target_)
        {
          pending_tag_estop_to_wait_target_ = false;
          changeFSMExecState(WAIT_TARGET, "TAG_DONE");
        }
        else if (enable_fail_safe_)
        {
          pending_estop_global_replan_ = true;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }
    }

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::isOdomBodyInObstacle() const
  {
    if (!have_odom_)
      return false;

    const auto map = planner_manager_->grid_map_;
    Eigen::Vector3d p = odom_pos_;
    return map->getInflateOccupancyNoFootprint(p) > 0;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {
    if (!have_odom_)
      return false;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ < 1e-3)
      return planFromGlobalTraj(trial_times);

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(false, flag_random_poly_init))
        return true;
    }

    // Odom jump / stale warm-start ctrl pts: retry from polynomial init when body is free.
    if (!isOdomBodyInObstacle())
    {
      for (int i = 0; i < trial_times; i++)
      {
        if (callReboundReplan(true, flag_random_poly_init))
          return true;
      }
    }
    return false;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == WAIT_TARGET || exec_state_ == INIT ||
        info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
      {
        RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");
        enterEmergencyStop("SAFETY");
      }
      else if (exec_state_ == GEN_NEW_TRAJ)
      {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(),
          std::max(log_trace_period_ms_, 500),
          "Depth stale during GEN_NEW_TRAJ recovery (no re-estop)");
      }
      return;
    }

    if (exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // Use node ROS clock to match start_time_ (set with node_->now() in planner_manager).
    double t_cur = (node_->now() - info->start_time_).seconds();

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double odom_traj_xy_dist = (odom_pos_.head<2>() - p_cur.head<2>()).norm();
    if (odom_traj_xy_dist > odom_traj_mismatch_thresh_)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(),
        std::max(log_trace_period_ms_, 500),
        "Odom-traj mismatch %.3fm (>%.3fm), force replan (odom jump / stale traj).",
        odom_traj_xy_dist, odom_traj_mismatch_thresh_);
      changeFSMExecState(REPLAN_TRAJ, "ODOM_JUMP");
      return;
    }

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      Eigen::Vector3d p_chk = info->position_traj_.evaluateDeBoorT(t);
      p_chk(2) = odom_pos_(2);
      occ |= map->getInflateOccupancy(p_chk);

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          const bool odom_in_obstacle = isOdomBodyInObstacle();
          if (!odom_in_obstacle)
          {
            RCLCPP_WARN(
              node_->get_logger(),
              "Trajectory collision but odom body free (odom-traj dist=%.3fm, dt=%.3fs), replan.",
              odom_traj_xy_dist, t - t_cur);
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          else if (t - t_cur < emergency_time_)
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            enterEmergencyStop("SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    planner_manager_->setRobotPlanningZ(odom_pos_(2));
    start_pt_(2) = odom_pos_(2);
    start_vel_(2) = 0.0;
    start_acc_(2) = 0.0;
    local_target_pt_(2) = odom_pos_(2);
    local_target_vel_(2) = 0.0;

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();

      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      const Eigen::Vector3d traj_end = info->position_traj_.evaluateDeBoorT(info->duration_);
      const double dist_goal = (end_pt_ - odom_pos_).norm();
      RCLCPP_INFO(
        node_->get_logger(),
        "[bspline_publish] traj_id=%d dur=%.2fs odom=%s goal=%s dist_goal=%.3f traj_end=%s start_pt=%s %s",
        info->traj_id_, info->duration_,
        traj_utils::formatVec3(odom_pos_).c_str(),
        traj_utils::formatVec3(end_pt_).c_str(),
        dist_goal,
        traj_utils::formatVec3(traj_end).c_str(),
        traj_utils::formatVec3(start_pt_).c_str(),
        traj_utils::formatControlPointsSummary(pos_pts).c_str());

      /* publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::enterEmergencyStop(const string &pos_call, bool disable_fail_safe)
  {
    if (disable_fail_safe)
      enable_fail_safe_ = false;
    if (exec_state_ == EMERGENCY_STOP)
      return;

    changeFSMExecState(EMERGENCY_STOP, pos_call);
    if (have_odom_)
      callEmergencyStop(odom_pos_);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  double EGOReplanFSM::distToGlobalTrajXY(const Eigen::Vector3d &pos, double *nearest_t_out)
  {
    if (planner_manager_->global_data_.global_duration_ < 1e-3)
      return std::numeric_limits<double>::infinity();

    const double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = std::numeric_limits<double>::infinity();
    double nearest_t = 0.0;

    for (double t = 0.0; t <= planner_manager_->global_data_.global_duration_ + 1e-6; t += t_step)
    {
      const Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      const double dist = (pos_t.head<2>() - pos.head<2>()).norm();
      if (dist < dist_min)
      {
        dist_min = dist;
        nearest_t = t;
      }
    }

    if (nearest_t_out)
      *nearest_t_out = nearest_t;
    return dist_min;
  }

  bool EGOReplanFSM::maybeReplanGlobalAfterEstop()
  {
    if (!have_target_ || !have_odom_)
      return false;

    double nearest_t = 0.0;
    const double drift = distToGlobalTrajXY(odom_pos_, &nearest_t);

    if (drift <= global_replan_drift_thresh_)
    {
      planner_manager_->global_data_.last_progress_time_ = nearest_t;
      RCLCPP_INFO(
        node_->get_logger(),
        "[global_replan] skip drift_xy=%.3f <= thresh=%.3f nearest_t=%.2f",
        drift, global_replan_drift_thresh_, nearest_t);
      return false;
    }

    planner_manager_->setRobotPlanningZ(odom_pos_(2));
    const bool ok = planner_manager_->planGlobalTraj(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (ok)
    {
      constexpr double step_size_t = 0.1;
      const int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> global_traj(i_end);
      for (int i = 0; i < i_end; i++)
        global_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      visualization_->displayGlobalPathList(global_traj, 0.1, 0);
    }

    if (ok)
      RCLCPP_INFO(
        node_->get_logger(),
        "[global_replan] replan drift_xy=%.3f > thresh=%.3f odom=%s goal=%s",
        drift, global_replan_drift_thresh_,
        traj_utils::formatVec3(odom_pos_).c_str(),
        traj_utils::formatVec3(end_pt_).c_str());
    else
      RCLCPP_WARN(
        node_->get_logger(),
        "[global_replan] failed drift_xy=%.3f > thresh=%.3f",
        drift, global_replan_drift_thresh_);

    return ok;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

  bool EGOReplanFSM::isTagFollowing() const
  {
    return enable_tag_tracking_ &&
           (tag_track_state_ == TagTrackState::ACTIVE ||
            tag_track_state_ == TagTrackState::HOLD);
  }

  Eigen::Vector3d EGOReplanFSM::computeFollowGoal(const Eigen::Vector3d &tag_pos) const
  {
    Eigen::Vector3d goal = tag_pos;
    if (have_odom_)
      goal.z() = odom_pos_.z();
    return goal;
  }

  bool EGOReplanFSM::isCloseToTag(const Eigen::Vector3d &tag_pos) const
  {
    return (odom_pos_.head<2>() - tag_pos.head<2>()).norm() <= tag_stop_dist_;
  }

  bool EGOReplanFSM::shouldReplanForTagGoal(const Eigen::Vector3d &new_goal) const
  {
    if (tag_force_replan_)
      return true;
    if (!have_target_)
      return true;
    if ((new_goal.head<2>() - end_pt_.head<2>()).norm() > tag_update_min_dist_)
      return true;
    if (last_tag_replan_time_.nanoseconds() == 0)
      return true;
    return (node_->now() - last_tag_replan_time_).seconds() > tag_replan_min_period_;
  }

  void EGOReplanFSM::applyTagGoal(const Eigen::Vector3d &goal_wp)
  {
    init_pt_ = odom_pos_;
    planNextWaypoint(goal_wp);
    visualization_->displayGoalPoint(goal_wp, Eigen::Vector4d(0.0, 0.8, 0.2, 1.0), 0.35, 0);
    last_tag_replan_time_ = node_->now();
    tag_force_replan_ = false;
    RCLCPP_INFO(
        node_->get_logger(),
        "[tag_track] replan tag=%s goal=%s",
        traj_utils::formatVec3(tag_pos_last_).c_str(),
        traj_utils::formatVec3(goal_wp).c_str());
  }

  void EGOReplanFSM::finishTagTracking(const string &reason)
  {
    const double dist_tag =
        have_tag_pose_ ? (odom_pos_.head<2>() - tag_pos_last_.head<2>()).norm() : -1.0;
    RCLCPP_INFO(
        node_->get_logger(),
        "[tag_track] DONE (%s) odom=%s tag=%s dist_xy=%.3f -> EMERGENCY_STOP",
        reason.c_str(),
        traj_utils::formatVec3(odom_pos_).c_str(),
        traj_utils::formatVec3(tag_pos_last_).c_str(),
        dist_tag);
    tag_track_state_ = TagTrackState::DONE;
    have_target_ = false;
    pending_tag_estop_to_wait_target_ = true;
    enterEmergencyStop("TAG_DONE");
  }

  void EGOReplanFSM::processTagDetection(bool detected)
  {
    if (!enable_tag_tracking_)
      return;

    std::lock_guard<std::mutex> lock(tag_mutex_);
    tag_detected_ = detected;

    if (!detected)
    {
      if (tag_track_state_ == TagTrackState::ACTIVE)
      {
        tag_pos_frozen_ = tag_pos_last_;
        tag_track_state_ = TagTrackState::HOLD;
        tag_force_replan_ = true;
        const Eigen::Vector3d goal = computeFollowGoal(tag_pos_frozen_);
        RCLCPP_INFO(
            node_->get_logger(),
            "[tag_track] HOLD tag=%s goal=%s",
            traj_utils::formatVec3(tag_pos_frozen_).c_str(),
            traj_utils::formatVec3(goal).c_str());
        if (!isCloseToTag(tag_pos_frozen_) && shouldReplanForTagGoal(goal))
          applyTagGoal(goal);
      }
      return;
    }

    if (!have_tag_pose_)
      return;

    const bool resume = tag_track_state_ == TagTrackState::HOLD;
    const bool first_seen = tag_track_state_ == TagTrackState::NEVER_SEEN;
    if (first_seen)
      RCLCPP_INFO(node_->get_logger(), "[tag_track] ACTIVE (first detection)");
    else if (resume)
      RCLCPP_INFO(node_->get_logger(), "[tag_track] RESUME");

    tag_track_state_ = TagTrackState::ACTIVE;
    tag_force_replan_ = first_seen || resume;

    if (!isCloseToTag(tag_pos_last_))
    {
      const Eigen::Vector3d goal = computeFollowGoal(tag_pos_last_);
      if (shouldReplanForTagGoal(goal))
        applyTagGoal(goal);
    }
  }

  void EGOReplanFSM::updateTagTrackingOnExecTick()
  {
    std::lock_guard<std::mutex> lock(tag_mutex_);

    if (tag_track_state_ == TagTrackState::ACTIVE && tag_detected_ && have_tag_pose_)
    {
      if (isCloseToTag(tag_pos_last_))
      {
        const double dist = (odom_pos_.head<2>() - tag_pos_last_.head<2>()).norm();
        RCLCPP_INFO(
            node_->get_logger(),
            "[tag_track] close to tag dist_xy=%.3f thresh=%.3f",
            dist, tag_stop_dist_);
        finishTagTracking("close to tag");
        return;
      }
    }

    if (tag_track_state_ != TagTrackState::HOLD)
      return;

    if (isCloseToTag(tag_pos_frozen_))
    {
      finishTagTracking("close to frozen tag");
      return;
    }
  }

  void EGOReplanFSM::tagPoseCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (!enable_tag_tracking_)
      return;

    if (msg->header.frame_id != "global")
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "[tag_track] pose frame_id='%s' != 'global', ignored",
          msg->header.frame_id.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(tag_mutex_);
    tag_pos_last_(0) = msg->pose.position.x;
    tag_pos_last_(1) = msg->pose.position.y;
    tag_pos_last_(2) = msg->pose.position.z;
    have_tag_pose_ = true;

    if (!tag_detected_)
      return;

    if (tag_track_state_ == TagTrackState::NEVER_SEEN)
    {
      tag_track_state_ = TagTrackState::ACTIVE;
      tag_force_replan_ = true;
      RCLCPP_INFO(node_->get_logger(), "[tag_track] ACTIVE (first detection, pose)");
      if (!isCloseToTag(tag_pos_last_))
        applyTagGoal(computeFollowGoal(tag_pos_last_));
      return;
    }

    if (tag_track_state_ != TagTrackState::ACTIVE)
      return;

    if (isCloseToTag(tag_pos_last_))
      return;

    const Eigen::Vector3d goal = computeFollowGoal(tag_pos_last_);
    if (shouldReplanForTagGoal(goal))
      applyTagGoal(goal);
  }

  void EGOReplanFSM::tagDetectedCallback(const std::shared_ptr<const std_msgs::msg::Bool> &msg)
  {
    if (!enable_tag_tracking_)
      return;
    processTagDetection(msg->data);
  }

} // namespace ego_planner
