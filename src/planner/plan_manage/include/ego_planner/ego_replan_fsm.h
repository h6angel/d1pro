#ifndef _REBO_REPLAN_FSM_H_

#define _REBO_REPLAN_FSM_H_



#include <Eigen/Eigen>

#include <algorithm>

#include <iostream>

#include "nav_msgs/msg/path.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "std_msgs/msg/empty.hpp"

#include <mutex>

#include <vector>

#include "visualization_msgs/msg/marker.hpp"



#include "bspline_opt/bspline_optimizer.h"

#include "plan_env/grid_map.h"

#include "traj_utils/msg/bspline.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ego_planner/planner_manager.h"

#include "traj_utils/planning_visualization.h"



using std::vector;



namespace ego_planner

{



  class EGOReplanFSM

  {



  private:

    /* ---------- flag ---------- */

    enum FSM_EXEC_STATE

    {

      INIT,

      WAIT_TARGET,

      GEN_NEW_TRAJ,

      REPLAN_TRAJ,

      EXEC_TRAJ,

      EMERGENCY_STOP

    };

    /* planning utils */

    EGOPlannerManager::Ptr planner_manager_;

    PlanningVisualization::Ptr visualization_;



    /* parameters */

    double no_replan_thresh_, replan_thresh_, goal_reach_thresh_;

    int log_trace_period_ms_{500};

    double planning_horizen_, planning_horizen_time_;

    double emergency_time_;

    /// Traj hit within this horizon (s) + body in inflate + moving toward hit => EMERGENCY_STOP.
    double estop_imminent_time_{0.3};

    /// Below this XY speed (m/s), inflation-only contact prefers REPLAN over estop.
    double estop_min_approach_speed_{0.05};

    /// Safety callback: planFromCurrentTraj trial count before tiered estop/replan.
    int safety_replan_trials_{5};

    double global_replan_drift_thresh_;

    /// Trajectory safety check: arc-length step between segment raycasts (m).
    double collision_check_step_{0.05};

    bool enable_fail_safe_;



    /* AprilTag tracking (launch-time only) */

    enum class TagTrackState

    {

      NEVER_SEEN,

      ACTIVE,

      HOLD,

      DONE

    };

    bool enable_tag_tracking_{false};

    std::string tag_pose_topic_{"/apriltag/target_pose_global"};

    std::string tag_detected_topic_{"/apriltag/target_detected"};

    double tag_stop_dist_{0.25};

    double tag_update_min_dist_{0.08};

    double tag_replan_min_period_{0.5};

    TagTrackState tag_track_state_{TagTrackState::NEVER_SEEN};

    bool tag_detected_{false};

    bool have_tag_pose_{false};

    bool tag_force_replan_{false};

    Eigen::Vector3d tag_pos_last_{0.0, 0.0, 0.0};

    Eigen::Vector3d tag_pos_frozen_{0.0, 0.0, 0.0};

    rclcpp::Time last_tag_replan_time_{0, 0, RCL_ROS_TIME};

    std::mutex tag_mutex_;



    /* planning data */

    bool have_target_, have_odom_, have_new_target_;

    bool pending_estop_global_replan_{false};

  /// Tag tracking finish: EMERGENCY_STOP until odom speed drops, then WAIT_TARGET (not GEN_NEW_TRAJ).

    bool pending_tag_estop_to_wait_target_{false};

    FSM_EXEC_STATE exec_state_;

    int continously_called_times_{0};



    /// GEN_NEW_TRAJ recovery: backoff + cap to avoid exec-timer spin when plan fails.

    int gen_new_traj_fail_count_{0};

    rclcpp::Time gen_new_traj_next_attempt_{0, 0, RCL_ROS_TIME};

    int gen_new_traj_max_failures_{8};

    double gen_new_traj_backoff_base_sec_{0.25};

    double gen_new_traj_backoff_max_sec_{2.0};



    void resetGenNewTrajRetry();

    void onGenNewTrajPlanFailed();



    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state

    Eigen::Quaterniond odom_orient_;



    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state

    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state

    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state



    /* ROS utils */

    rclcpp::Node::SharedPtr node_;

    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;



    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_detected_sub_;



    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;



    /* helper functions */

    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method

    /// warm-start (optional) → poly init → random poly escape (always try last stage).
    bool callReboundReplanWithEscape(int trial_times, bool try_warm_start);

    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method

    void enterEmergencyStop(const string &pos_call, bool disable_fail_safe = false);

    /// First plan / emergency: polynomial init from /odom to local target.

    bool planFromGlobalTraj(const int trial_times = 1);

    /// Local replan: warm-start from previous B-spline at t_cur; pin first ctrl pts to /odom.

    bool planFromCurrentTraj(const int trial_times = 1);

    bool isOdomBodyInObstacle() const;

    /// Tier-2: after replan escape fails, decide EMERGENCY_STOP vs REPLAN_TRAJ.
    bool shouldEmergencyStopOnTrajHit(double dt_to_hit, const Eigen::Vector3d &hit_pos) const;

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */

    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    void printFSMExecState();



    void planNextWaypoint(const Eigen::Vector3d next_wp);

    void getLocalTarget();

    double distToGlobalTrajXY(const Eigen::Vector3d &pos, double *nearest_t_out = nullptr);

    bool maybeReplanGlobalAfterEstop();



    bool isTagFollowing() const;

    Eigen::Vector3d computeFollowGoal(const Eigen::Vector3d &tag_pos) const;

    bool isCloseToTag(const Eigen::Vector3d &tag_pos) const;

    bool shouldReplanForTagGoal(const Eigen::Vector3d &new_goal) const;

    void applyTagGoal(const Eigen::Vector3d &goal_wp);

    void finishTagTracking(const string &reason);

    void processTagDetection(bool detected);

    void updateTagTrackingOnExecTick();



    /* ROS functions */

    void execFSMCallback();

    void checkCollisionCallback();

    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);

    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);

    void tagPoseCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);

    void tagDetectedCallback(const std::shared_ptr<const std_msgs::msg::Bool> &msg);



  public:

    EGOReplanFSM(/* args */)

    {

    }

    ~EGOReplanFSM()

    {

    }



    void init(rclcpp::Node::SharedPtr &node);



    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };



} // namespace ego_planner



#endif


