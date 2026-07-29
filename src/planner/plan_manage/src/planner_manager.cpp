// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <thread>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

namespace
{
void flattenStateZ(const double z_ref, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc)
{
  pos(2) = z_ref;
  vel(2) = 0.0;
  acc(2) = 0.0;
}

void flattenPointZ(const double z_ref, Eigen::Vector3d &p) { p(2) = z_ref; }

void flattenPointSetZ(const double z_ref, std::vector<Eigen::Vector3d> &pts)
{
  for (auto &p : pts)
    p(2) = z_ref;
}

void flattenControlPointsZ(const double z_ref, Eigen::MatrixXd &ctrl_pts)
{
  for (int i = 0; i < ctrl_pts.cols(); ++i)
    ctrl_pts(2, i) = z_ref;
}

/// Douglas–Peucker on XY; keep Z of retained samples.
std::vector<Eigen::Vector3d> simplifyPolylineXY(const std::vector<Eigen::Vector3d> &in, double eps)
{
  if (in.size() <= 2 || eps <= 1e-6)
    return in;

  std::vector<char> keep(in.size(), 0);
  keep.front() = 1;
  keep.back() = 1;

  std::function<void(int, int)> rdp = [&](int i0, int i1) {
    if (i1 <= i0 + 1)
      return;
    const Eigen::Vector2d a = in[static_cast<size_t>(i0)].head<2>();
    const Eigen::Vector2d b = in[static_cast<size_t>(i1)].head<2>();
    const Eigen::Vector2d ab = b - a;
    const double ab2 = ab.squaredNorm();
    double max_d = 0.0;
    int imax = i0;
    for (int i = i0 + 1; i < i1; ++i)
    {
      double d = 0.0;
      if (ab2 < 1e-12)
        d = (in[static_cast<size_t>(i)].head<2>() - a).norm();
      else
      {
        const double t = std::clamp(
            (in[static_cast<size_t>(i)].head<2>() - a).dot(ab) / ab2, 0.0, 1.0);
        d = (in[static_cast<size_t>(i)].head<2>() - (a + t * ab)).norm();
      }
      if (d > max_d)
      {
        max_d = d;
        imax = i;
      }
    }
    if (max_d > eps)
    {
      keep[static_cast<size_t>(imax)] = 1;
      rdp(i0, imax);
      rdp(imax, i1);
    }
  };
  rdp(0, static_cast<int>(in.size()) - 1);

  std::vector<Eigen::Vector3d> out;
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i)
    if (keep[i])
      out.push_back(in[i]);
  return out;
}

bool segmentInflateFree(const GridMap::Ptr &map, const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                        double step)
{
  if (!map)
    return false;
  const double len = (b - a).norm();
  if (len < 1e-6)
    return map->getInflateOccupancy(a) == 0;
  step = std::max(step, 0.05);
  const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
  for (int i = 0; i <= n; ++i)
  {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const Eigen::Vector3d p = a + t * (b - a);
    if (map->getInflateOccupancy(p) != 0)
      return false;
  }
  return true;
}

void densifyWaypoints(const std::vector<Eigen::Vector3d> &points, double dist_thresh,
                      std::vector<Eigen::Vector3d> &inter_points)
{
  inter_points.clear();
  if (points.empty())
    return;
  for (size_t i = 0; i + 1 < points.size(); ++i)
  {
    inter_points.push_back(points[i]);
    const double dist = (points[i + 1] - points[i]).norm();
    if (dist > dist_thresh)
    {
      const int id_num = static_cast<int>(std::floor(dist / dist_thresh)) + 1;
      for (int j = 1; j < id_num; ++j)
      {
        inter_points.push_back(points[i] * (1.0 - double(j) / id_num) +
                               points[i + 1] * (double(j) / id_num));
      }
    }
  }
  inter_points.push_back(points.back());
}

double wrapPi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

void appendIfFar(std::vector<Eigen::Vector3d> &out, const Eigen::Vector3d &p, double min_dist)
{
  if (out.empty() || (p.head<2>() - out.back().head<2>()).norm() >= min_dist)
    out.push_back(p);
}
} // namespace

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node_ = node;

    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/use_robot_z_planning", true);
    node->declare_parameter("manager/global_astar_enable", true);
    node->declare_parameter("manager/global_astar_step", 0.2);
    node->declare_parameter("manager/global_astar_timeout", 0.5);
    node->declare_parameter("manager/global_astar_fallback_straight", true);
    node->declare_parameter("manager/global_astar_simplify_eps", 0.25);
    node->declare_parameter("manager/global_astar_insert_dist", 4.0);
    node->declare_parameter("manager/hybrid_enable", true);
    node->declare_parameter("manager/hybrid_max_curvature", 0.83);
    node->declare_parameter("manager/hybrid_corner_angle_thresh", 0.5);
    node->declare_parameter("manager/hybrid_arc_sample_step", 0.2);
    node->declare_parameter("manager/hybrid_max_arc_points", 30);
    node->declare_parameter("manager/hybrid_use_odom_start_yaw", true);
    node->declare_parameter("manager/hybrid_blend_start_yaw", true);
    node->declare_parameter("manager/hybrid_align_yaw_thresh", 0.4);
    node->declare_parameter("manager/hybrid_blend_occ_check", true);
    node->declare_parameter("manager/hybrid_blend_r_shrink_tries", 2);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_robot_z_planning", use_robot_z_planning_);
    node->get_parameter("manager/global_astar_enable", global_astar_enable_);
    node->get_parameter("manager/global_astar_step", global_astar_step_);
    node->get_parameter("manager/global_astar_timeout", global_astar_timeout_);
    node->get_parameter("manager/global_astar_fallback_straight", global_astar_fallback_straight_);
    node->get_parameter("manager/global_astar_simplify_eps", global_astar_simplify_eps_);
    node->get_parameter("manager/global_astar_insert_dist", global_astar_insert_dist_);
    node->get_parameter("manager/hybrid_enable", hybrid_enable_);
    node->get_parameter("manager/hybrid_max_curvature", hybrid_max_curvature_);
    node->get_parameter("manager/hybrid_corner_angle_thresh", hybrid_corner_angle_thresh_);
    node->get_parameter("manager/hybrid_arc_sample_step", hybrid_arc_sample_step_);
    node->get_parameter("manager/hybrid_max_arc_points", hybrid_max_arc_points_);
    node->get_parameter("manager/hybrid_use_odom_start_yaw", hybrid_use_odom_start_yaw_);
    node->get_parameter("manager/hybrid_blend_start_yaw", hybrid_blend_start_yaw_);
    node->get_parameter("manager/hybrid_align_yaw_thresh", hybrid_align_yaw_thresh_);
    node->get_parameter("manager/hybrid_blend_occ_check", hybrid_blend_occ_check_);
    node->get_parameter("manager/hybrid_blend_r_shrink_tries", hybrid_blend_r_shrink_tries_);

    global_astar_step_ = std::max(global_astar_step_, 0.05);
    global_astar_timeout_ = std::max(global_astar_timeout_, 0.05);
    global_astar_simplify_eps_ = std::max(global_astar_simplify_eps_, 0.0);
    global_astar_insert_dist_ = std::max(global_astar_insert_dist_, 0.5);
    hybrid_max_curvature_ = std::max(hybrid_max_curvature_, 0.05);
    hybrid_corner_angle_thresh_ = std::max(hybrid_corner_angle_thresh_, 0.05);
    hybrid_arc_sample_step_ = std::max(hybrid_arc_sample_step_, 0.05);
    hybrid_max_arc_points_ = std::max(hybrid_max_arc_points_, 4);
    hybrid_align_yaw_thresh_ = std::max(hybrid_align_yaw_thresh_, 0.05);
    hybrid_blend_r_shrink_tries_ = std::clamp(hybrid_blend_r_shrink_tries_, 0, 4);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    // Global A*: larger pool so start→goal (+ detour) fits; Z thin (planar search).
    Eigen::Vector3d map_ori, map_size;
    grid_map_->getRegion(map_ori, map_size);
    const double cover = std::max(map_size(0), map_size(1)) + 4.0;
    int pool_xy = static_cast<int>(std::ceil(cover / global_astar_step_)) + 4;
    pool_xy = std::clamp(pool_xy, 120, 420);
    global_a_star_.reset(new AStar);
    global_a_star_->initGridMap(grid_map_, Eigen::Vector3i(pool_xy, pool_xy, 5));
    RCLCPP_INFO(
        node_->get_logger(),
        "[global_astar] enable=%d step=%.2f timeout=%.2fs pool=%dx%dx5 fallback_straight=%d",
        global_astar_enable_ ? 1 : 0, global_astar_step_, global_astar_timeout_, pool_xy, pool_xy,
        global_astar_fallback_straight_ ? 1 : 0);
    RCLCPP_INFO(
        node_->get_logger(),
        "[hybrid_L] enable=%d kappa_max=%.3f corner_thresh=%.2f arc_step=%.2f "
        "use_odom_yaw=%d blend_start=%d align_thresh=%.2f occ_check=%d r_shrink=%d",
        hybrid_enable_ ? 1 : 0, hybrid_max_curvature_, hybrid_corner_angle_thresh_,
        hybrid_arc_sample_step_, hybrid_use_odom_start_yaw_ ? 1 : 0,
        hybrid_blend_start_yaw_ ? 1 : 0, hybrid_align_yaw_thresh_,
        hybrid_blend_occ_check_ ? 1 : 0, hybrid_blend_r_shrink_tries_);

    visualization_ = vis;
  }

  void EGOPlannerManager::setRobotPlanningZ(double z)
  {
    if (!use_robot_z_planning_)
    {
      bspline_optimizer_->clearPlanningZ();
      return;
    }
    bspline_optimizer_->setPlanningZ(z);
  }

  void EGOPlannerManager::updateRobotPosition(const Eigen::Vector3d &pos)
  {
    if (grid_map_)
      grid_map_->updateRobotPosition(pos);
  }

  void EGOPlannerManager::updateRobotPose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q)
  {
    if (grid_map_)
      grid_map_->updateRobotPose(pos, q);

    // Body +Z horizontal projection (same convention as d1_planner_bridge).
    const double n2 = q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
    if (std::isfinite(n2) && n2 > 1e-8)
    {
      const double inv_n = 1.0 / std::sqrt(n2);
      const double w = q.w() * inv_n;
      const double x = q.x() * inv_n;
      const double y = q.y() * inv_n;
      const double z = q.z() * inv_n;
      const double zx = 2.0 * (x * z + w * y);
      const double zy = 2.0 * (y * z - w * x);
      const double horiz = std::hypot(zx, zy);
      if (std::isfinite(horiz) && horiz > 1e-6)
      {
        robot_yaw_ = std::atan2(zy, zx);
        have_robot_yaw_ = std::isfinite(robot_yaw_);
      }
    }
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[replan %d]==============================================\033[0m\n", count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    if (bspline_optimizer_->usePlanningZ())
    {
      const double z_ref = bspline_optimizer_->getPlanningZ();
      flattenStateZ(z_ref, start_pt, start_vel, start_acc);
      flattenPointZ(z_ref, local_target_pt);
      local_target_vel(2) = 0.0;
      flag_randomPolyTraj = false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          const double z_ref = bspline_optimizer_->usePlanningZ() ? bspline_optimizer_->getPlanningZ() : start_pt(2);
          Eigen::Vector3d mid = (start_pt + local_target_pt) / 2;
          Eigen::Vector2d seg_xy = (local_target_pt - start_pt).head<2>();
          if (seg_xy.norm() < 1e-3)
            seg_xy = Eigen::Vector2d(1.0, 0.0);
          else
            seg_xy.normalize();
          const Eigen::Vector3d horizen_dir(-seg_xy(1), seg_xy(0), 0.0);
          const double rand_scale = (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * 0.8 *
                                    (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::Vector3d random_inserted_pt = mid + horizen_dir * rand_scale;
          random_inserted_pt(2) = z_ref;
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();
        t_cur = std::max(0.0, std::min(t_cur, local_data_.duration_));

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        // Need >=2 samples for arc-length interpolation; otherwise size()-2 underflows and segfaults.
        if (pseudo_arc_length.size() < 2 || segment_point.size() < 2)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while (id + 1 < pseudo_arc_length.size() && sample_length <= pseudo_arc_length.back())
          {
            const double seg_len = pseudo_arc_length[id + 1] - pseudo_arc_length[id];
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1] && seg_len > 1e-9)
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / seg_len * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / seg_len * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    if (bspline_optimizer_->usePlanningZ())
      flattenPointSetZ(bspline_optimizer_->getPlanningZ(), point_set);

    // 将轨迹变为B样条轨迹
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // Hybrid warm-start: stitch from previous traj, anchor start to measured odom (fixed during L-BFGS).
    if (!flag_polyInit)
    {
      constexpr int k_pin_start_cps = 3;
      for (int i = 0; i < k_pin_start_cps && i < ctrl_pts.cols(); ++i)
        ctrl_pts.col(i) = start_pt + start_vel * (static_cast<double>(i) * ts);
    }

    if (bspline_optimizer_->usePlanningZ())
      flattenControlPointsZ(bspline_optimizer_->getPlanningZ(), ctrl_pts);

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    // 计算时间差并更新时间
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;

    flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    t_opt = rclcpp::Clock().now() - t_start;
    visualization_->displayInitPathList(point_set, 0.2, 0);

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    if (bspline_optimizer_->usePlanningZ())
      flattenControlPointsZ(bspline_optimizer_->getPlanningZ(), ctrl_pts);

    t_start = rclcpp::Clock().now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    {
      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;

    // save planned results
    updateTrajInfo(pos, node_->now());

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();

    const double arc_t_step = pp_.planning_horizen_ / 20.0 / std::max(pp_.max_vel_, 0.1);
    global_data_.setGlobalTraj(gl_traj, time_now, arc_t_step);

    return true;
  }

  bool EGOPlannerManager::buildGlobalWaypoints(const Eigen::Vector3d &start, const Eigen::Vector3d &end,
                                               std::vector<Eigen::Vector3d> &waypoints)
  {
    waypoints.clear();

    const double check_step = std::min(global_astar_step_, 0.15);
    const bool straight_free = segmentInflateFree(grid_map_, start, end, check_step);

    if (!global_astar_enable_ || straight_free)
    {
      waypoints.push_back(start);
      waypoints.push_back(end);
      if (straight_free && global_astar_enable_)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "[global_astar] skip search: straight line free (len=%.2f)",
                    (end - start).norm());
      }
      applyHybridCurvatureL(waypoints);
      return true;
    }

    if (!global_a_star_)
    {
      RCLCPP_WARN(node_->get_logger(), "[global_astar] planner not initialized");
      if (global_astar_fallback_straight_)
      {
        waypoints.push_back(start);
        waypoints.push_back(end);
        applyHybridCurvatureL(waypoints);
        return true;
      }
      return false;
    }

    const bool ok = global_a_star_->AstarSearch(global_astar_step_, start, end, global_astar_timeout_);
    if (!ok)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[global_astar] search failed start=(%.2f,%.2f) end=(%.2f,%.2f) fallback=%d",
                  start(0), start(1), end(0), end(1), global_astar_fallback_straight_ ? 1 : 0);
      if (global_astar_fallback_straight_)
      {
        waypoints.push_back(start);
        waypoints.push_back(end);
        applyHybridCurvatureL(waypoints);
        return true;
      }
      return false;
    }

    waypoints = global_a_star_->getPath();
    if (waypoints.size() < 2)
    {
      waypoints.clear();
      waypoints.push_back(start);
      waypoints.push_back(end);
    }
    else
    {
      // Snap endpoints to exact start/goal (search may adjust occupied cells).
      waypoints.front() = start;
      waypoints.back() = end;
      if (bspline_optimizer_->usePlanningZ())
      {
        const double z_ref = bspline_optimizer_->getPlanningZ();
        flattenPointSetZ(z_ref, waypoints);
      }
      waypoints = simplifyPolylineXY(waypoints, global_astar_simplify_eps_);
    }

    RCLCPP_INFO(node_->get_logger(),
                "[global_astar] ok waypoints=%zu len≈%.2f simplify_eps=%.2f",
                waypoints.size(), (end - start).norm(), global_astar_simplify_eps_);

    applyHybridCurvatureL(waypoints);

    if (visualization_)
    {
      std::vector<std::vector<Eigen::Vector3d>> vis_paths;
      vis_paths.push_back(waypoints);
      visualization_->displayAStarList(vis_paths, 0);
    }

    return true;
  }

  void EGOPlannerManager::applyHybridCurvatureL(std::vector<Eigen::Vector3d> &waypoints)
  {
    if (!hybrid_enable_ || waypoints.size() < 2)
      return;

    const double z_ref = waypoints.front()(2);
    const double R_nom = 1.0 / hybrid_max_curvature_;
    const double min_keep = 0.5 * hybrid_arc_sample_step_;
    const size_t n_in = waypoints.size();

    // --- Optional: blend from current body yaw onto first segment ---
    if (hybrid_use_odom_start_yaw_ && hybrid_blend_start_yaw_ && have_robot_yaw_)
    {
      const Eigen::Vector2d d0 = (waypoints[1] - waypoints[0]).head<2>();
      if (d0.norm() > 1e-3)
      {
        const double path_yaw = std::atan2(d0.y(), d0.x());
        const double dyaw = wrapPi(path_yaw - robot_yaw_);
        if (std::abs(dyaw) > hybrid_align_yaw_thresh_)
        {
          const Eigen::Vector2d p0 = waypoints[0].head<2>();
          const Eigen::Vector2d dir(std::cos(robot_yaw_), std::sin(robot_yaw_));
          Eigen::Vector2d n(-dir.y(), dir.x());
          if (dyaw < 0.0)
            n = -n;
          const double sweep = dyaw;

          auto arcFree = [&](double R, int n_samp) -> bool {
            if (!hybrid_blend_occ_check_ || !grid_map_)
              return true;
            const Eigen::Vector2d center = p0 + n * R;
            const double a0 = std::atan2((p0 - center).y(), (p0 - center).x());
            for (int k = 0; k <= n_samp; ++k)
            {
              const double a = a0 + sweep * static_cast<double>(k) / static_cast<double>(n_samp);
              Eigen::Vector3d p(center.x() + R * std::cos(a), center.y() + R * std::sin(a), z_ref);
              const int occ = grid_map_->getInflateOccupancy(p);
              if (occ != 0)
                return false;
            }
            return true;
          };

          // Try nominal R, then progressively smaller radii; skip blend if all collide.
          const double shrink_factors[] = {1.0, 0.7, 0.45};
          const int n_try = 1 + std::min(hybrid_blend_r_shrink_tries_, 2);
          bool applied = false;
          for (int ti = 0; ti < n_try; ++ti)
          {
            const double R = R_nom * shrink_factors[ti];
            if (R < 0.15)
              continue;
            const int n_samp = std::clamp(
                static_cast<int>(std::ceil(R * std::abs(sweep) / hybrid_arc_sample_step_)),
                2, hybrid_max_arc_points_);
            if (!arcFree(R, n_samp))
            {
              RCLCPP_WARN(
                  node_->get_logger(),
                  "[hybrid_L] start_yaw_blend R=%.2f occupied, try shrink/skip", R);
              continue;
            }

            const Eigen::Vector2d center = p0 + n * R;
            const double a0 = std::atan2((p0 - center).y(), (p0 - center).x());
            std::vector<Eigen::Vector3d> blended;
            blended.reserve(waypoints.size() + static_cast<size_t>(n_samp) + 2);
            for (int k = 0; k <= n_samp; ++k)
            {
              const double a = a0 + sweep * static_cast<double>(k) / static_cast<double>(n_samp);
              Eigen::Vector3d p(center.x() + R * std::cos(a), center.y() + R * std::sin(a), z_ref);
              appendIfFar(blended, p, min_keep);
            }
            for (size_t i = 1; i < waypoints.size(); ++i)
              appendIfFar(blended, waypoints[i], min_keep);
            waypoints.swap(blended);
            RCLCPP_INFO(node_->get_logger(),
                        "[hybrid_L] start_yaw_blend dyaw=%.2f rad R=%.2f samples=%d",
                        dyaw, R, n_samp);
            applied = true;
            break;
          }
          if (!applied)
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "[hybrid_L] start_yaw_blend skipped (arc occupied) dyaw=%.2f — keep polyline",
                dyaw);
          }
        }
      }
    }

    if (waypoints.size() < 3)
      return;

    // --- Round sharp corners with constant-curvature fillets ---
    std::vector<Eigen::Vector3d> out;
    out.reserve(waypoints.size() * 2);
    out.push_back(waypoints.front());

    int corners_rounded = 0;
    for (size_t i = 1; i + 1 < waypoints.size(); ++i)
    {
      const Eigen::Vector3d &A = out.back();
      const Eigen::Vector3d &B = waypoints[i];
      const Eigen::Vector3d &C = waypoints[i + 1];

      Eigen::Vector2d d1 = (B - A).head<2>();
      Eigen::Vector2d d2 = (C - B).head<2>();
      const double len1 = d1.norm();
      const double len2 = d2.norm();
      if (len1 < 1e-4 || len2 < 1e-4)
      {
        appendIfFar(out, B, min_keep);
        continue;
      }
      d1 /= len1;
      d2 /= len2;

      const double yaw1 = std::atan2(d1.y(), d1.x());
      const double yaw2 = std::atan2(d2.y(), d2.x());
      const double turn = wrapPi(yaw2 - yaw1);
      if (std::abs(turn) < hybrid_corner_angle_thresh_)
      {
        appendIfFar(out, B, min_keep);
        continue;
      }

      const double half = 0.5 * std::abs(turn);
      const double tan_half = std::tan(half);
      if (tan_half < 1e-4)
      {
        appendIfFar(out, B, min_keep);
        continue;
      }

      double R = R_nom;
      double t_len = R * tan_half;
      const double max_t = 0.45 * std::min(len1, len2);
      if (t_len > max_t)
      {
        t_len = max_t;
        R = t_len / tan_half;
      }
      if (R < 0.05 || t_len < hybrid_arc_sample_step_ * 0.5)
      {
        appendIfFar(out, B, min_keep);
        continue;
      }

      const Eigen::Vector2d entry = B.head<2>() - d1 * t_len;
      const Eigen::Vector2d exitp = B.head<2>() + d2 * t_len;
      Eigen::Vector2d n_in(-d1.y(), d1.x());
      if (turn < 0.0)
        n_in = -n_in;
      const Eigen::Vector2d center = entry + n_in * R;
      const double a0 = std::atan2((entry - center).y(), (entry - center).x());
      double a1 = std::atan2((exitp - center).y(), (exitp - center).x());
      double sweep = wrapPi(a1 - a0);
      if (turn > 0.0 && sweep < 0.0)
        sweep += 2.0 * M_PI;
      if (turn < 0.0 && sweep > 0.0)
        sweep -= 2.0 * M_PI;

      const int n_samp = std::clamp(
          static_cast<int>(std::ceil(R * std::abs(sweep) / hybrid_arc_sample_step_)),
          2, hybrid_max_arc_points_);

      appendIfFar(out, Eigen::Vector3d(entry.x(), entry.y(), z_ref), min_keep);
      for (int k = 1; k < n_samp; ++k)
      {
        const double a = a0 + sweep * static_cast<double>(k) / static_cast<double>(n_samp);
        appendIfFar(out,
                    Eigen::Vector3d(center.x() + R * std::cos(a), center.y() + R * std::sin(a), z_ref),
                    min_keep);
      }
      appendIfFar(out, Eigen::Vector3d(exitp.x(), exitp.y(), z_ref), min_keep);
      ++corners_rounded;
    }
    appendIfFar(out, waypoints.back(), min_keep);

    if (bspline_optimizer_->usePlanningZ())
      flattenPointSetZ(bspline_optimizer_->getPlanningZ(), out);

    waypoints.swap(out);
    RCLCPP_INFO(node_->get_logger(),
                "[hybrid_L] corners_rounded=%d waypoints %zu -> %zu kappa_max=%.3f",
                corners_rounded, n_in, waypoints.size(), hybrid_max_curvature_);
  }

  bool EGOPlannerManager::fitGlobalPolynomial(const std::vector<Eigen::Vector3d> &waypoints,
                                              const Eigen::Vector3d &start_vel, const Eigen::Vector3d &end_vel,
                                              const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_acc)
  {
    if (waypoints.size() < 2)
      return false;

    std::vector<Eigen::Vector3d> inter_points;
    densifyWaypoints(waypoints, global_astar_insert_dist_, inter_points);

    const int pt_num = static_cast<int>(inter_points.size());
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[static_cast<size_t>(i)];

    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / std::max(pp_.max_vel_, 0.05);

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(
          waypoints.front(), start_vel, start_acc, waypoints.back(), end_vel, end_acc, time(0));
    else
      return false;

    const double arc_t_step = pp_.planning_horizen_ / 20.0 / std::max(pp_.max_vel_, 0.1);
    global_data_.setGlobalTraj(gl_traj, node_->now(), arc_t_step);
    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    Eigen::Vector3d start = start_pos;
    Eigen::Vector3d end = end_pos;
    Eigen::Vector3d sv = start_vel;
    Eigen::Vector3d ev = end_vel;
    Eigen::Vector3d sa = start_acc;
    Eigen::Vector3d ea = end_acc;
    if (bspline_optimizer_->usePlanningZ())
    {
      const double z_ref = bspline_optimizer_->getPlanningZ();
      flattenPointZ(z_ref, start);
      flattenPointZ(z_ref, end);
      sv(2) = 0.0;
      ev(2) = 0.0;
      sa(2) = 0.0;
      ea(2) = 0.0;
    }

    std::vector<Eigen::Vector3d> waypoints;
    if (!buildGlobalWaypoints(start, end, waypoints))
      return false;

    return fitGlobalPolynomial(waypoints, sv, ev, sa, ea);
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
