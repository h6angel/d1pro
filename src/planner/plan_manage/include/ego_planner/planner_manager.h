#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    void setRobotPlanningZ(double z);
    void updateRobotPosition(const Eigen::Vector3d &pos);
    void updateRobotPose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // Node handle: trajectory timing (start_time_) must use the node ROS clock
    // (node_->now()) so it stays consistent with FSM / traj_server and respects use_sim_time.
    rclcpp::Node::SharedPtr node_;

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_;
    AStar::Ptr global_a_star_;

    int continous_failures_count_{0};
    bool use_robot_z_planning_{true};

    bool global_astar_enable_{true};
    double global_astar_step_{0.2};
    double global_astar_timeout_{0.5};
    bool global_astar_fallback_straight_{true};
    double global_astar_simplify_eps_{0.25};
    double global_astar_insert_dist_{4.0};

    // Scheme L: curvature fillets on A* polyline (non-holonomic soft constraint)
    bool hybrid_enable_{true};
    double hybrid_max_curvature_{0.83};
    double hybrid_corner_angle_thresh_{0.5};
    double hybrid_arc_sample_step_{0.2};
    int hybrid_max_arc_points_{30};
    bool hybrid_use_odom_start_yaw_{true};
    bool hybrid_blend_start_yaw_{true};
    double hybrid_align_yaw_thresh_{0.4};

    bool have_robot_yaw_{false};
    double robot_yaw_{0.0};

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    /// Build waypoint list for global poly: A* path (optional) then densify / simplify.
    bool buildGlobalWaypoints(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                              std::vector<Eigen::Vector3d> &waypoints);

    /// Scheme L: round sharp corners + optional start-yaw blend (mutates waypoints).
    void applyHybridCurvatureL(std::vector<Eigen::Vector3d> &waypoints);

    bool fitGlobalPolynomial(const std::vector<Eigen::Vector3d> &waypoints,
                             const Eigen::Vector3d &start_vel, const Eigen::Vector3d &end_vel,
                             const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_acc);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
