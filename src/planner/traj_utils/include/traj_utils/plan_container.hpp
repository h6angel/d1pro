#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace ego_planner
{

  class GlobalTrajData
  {
  private:
    vector<double> arc_s_;
    vector<double> arc_t_;
    vector<double> arc_x_;
    vector<double> arc_y_;
    double total_arc_length_ = 0.0;
    bool arc_table_valid_ = false;

    void buildArcLengthTable(double t_step)
    {
      arc_s_.clear();
      arc_t_.clear();
      arc_x_.clear();
      arc_y_.clear();
      total_arc_length_ = 0.0;
      arc_table_valid_ = false;

      if (global_duration_ < 1e-3)
        return;

      if (t_step < 1e-6)
        t_step = std::max(0.05, global_duration_ / 500.0);

      double s = 0.0;
      Eigen::Vector3d prev = global_traj_.evaluate(0.0);
      arc_s_.push_back(0.0);
      arc_t_.push_back(0.0);
      arc_x_.push_back(prev(0));
      arc_y_.push_back(prev(1));

      for (double t = t_step; t <= global_duration_ + 1e-6; t += t_step)
      {
        const double t_clip = std::min(t, global_duration_);
        const Eigen::Vector3d cur = global_traj_.evaluate(t_clip);
        s += (cur.head<2>() - prev.head<2>()).norm();
        arc_s_.push_back(s);
        arc_t_.push_back(t_clip);
        arc_x_.push_back(cur(0));
        arc_y_.push_back(cur(1));
        prev = cur;
        if (t_clip >= global_duration_ - 1e-9)
          break;
      }

      total_arc_length_ = s;
      arc_table_valid_ = arc_s_.size() >= 2;
    }

  public:
    PolynomialTraj global_traj_;
    vector<UniformBspline> local_traj_;

    double global_duration_;
    rclcpp::Time global_start_time_;
    double local_start_time_, local_end_time_;
    double time_increase_;
    double last_time_inc_;
    double last_progress_time_;

    GlobalTrajData(/* args */) {}

    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    void setGlobalTraj(const PolynomialTraj &traj, const rclcpp::Time &time, double arc_sample_t_step = 0.0)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;

      buildArcLengthTable(arc_sample_t_step);
    }

    bool arcTableValid() const { return arc_table_valid_; }

    double totalArcLength() const { return total_arc_length_; }

    double distToTrajXY(const Eigen::Vector3d &pos, double *nearest_t_out = nullptr,
                        double *nearest_s_out = nullptr)
    {
      if (!arc_table_valid_)
      {
        if (global_duration_ < 1e-3)
          return std::numeric_limits<double>::infinity();

        const Eigen::Vector3d p0 = global_traj_.evaluate(0.0);
        const double dist = (p0.head<2>() - pos.head<2>()).norm();
        if (nearest_t_out)
          *nearest_t_out = 0.0;
        if (nearest_s_out)
          *nearest_s_out = 0.0;
        return dist;
      }

      const double px = pos(0);
      const double py = pos(1);
      double dist_min = std::numeric_limits<double>::infinity();
      double best_t = 0.0;
      double best_s = 0.0;

      for (size_t i = 0; i + 1 < arc_s_.size(); ++i)
      {
        const double ax = arc_x_[i];
        const double ay = arc_y_[i];
        const double bx = arc_x_[i + 1];
        const double by = arc_y_[i + 1];
        const double dx = bx - ax;
        const double dy = by - ay;
        const double seg_len_sq = dx * dx + dy * dy;

        double u = 0.0;
        if (seg_len_sq > 1e-12)
        {
          u = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq;
          u = std::max(0.0, std::min(1.0, u));
        }

        const double cx = ax + u * dx;
        const double cy = ay + u * dy;
        const double dist = std::hypot(px - cx, py - cy);

        if (dist < dist_min)
        {
          dist_min = dist;
          best_s = arc_s_[i] + u * (arc_s_[i + 1] - arc_s_[i]);
          best_t = arc_t_[i] + u * (arc_t_[i + 1] - arc_t_[i]);
        }
      }

      if (nearest_t_out)
        *nearest_t_out = best_t;
      if (nearest_s_out)
        *nearest_s_out = best_s;
      return dist_min;
    }

    bool queryAtArcS(double s, Eigen::Vector3d &pos, double &t_out)
    {
      if (!arc_table_valid_ || arc_s_.empty())
        return false;

      s = std::max(0.0, std::min(s, total_arc_length_));

      if (arc_s_.size() == 1)
      {
        t_out = arc_t_[0];
        pos = global_traj_.evaluate(t_out);
        return true;
      }

      auto it = std::lower_bound(arc_s_.begin(), arc_s_.end(), s);
      if (it == arc_s_.begin())
      {
        t_out = arc_t_[0];
        pos = global_traj_.evaluate(t_out);
        return true;
      }
      if (it == arc_s_.end())
      {
        t_out = arc_t_.back();
        pos = global_traj_.evaluate(t_out);
        return true;
      }

      const size_t i1 = static_cast<size_t>(it - arc_s_.begin());
      const size_t i0 = i1 - 1;
      const double ds = arc_s_[i1] - arc_s_[i0];
      const double u = ds > 1e-12 ? (s - arc_s_[i0]) / ds : 0.0;
      t_out = arc_t_[i0] + u * (arc_t_[i1] - arc_t_[i0]);
      pos = global_traj_.evaluate(t_out);
      return true;
    }

    double sAtTime(double t)
    {
      if (!arc_table_valid_ || arc_t_.empty())
        return 0.0;

      t = std::max(0.0, std::min(t, global_duration_));
      auto it = std::lower_bound(arc_t_.begin(), arc_t_.end(), t);
      if (it == arc_t_.begin())
        return arc_s_.front();
      if (it == arc_t_.end())
        return arc_s_.back();

      const size_t i1 = static_cast<size_t>(it - arc_t_.begin());
      const size_t i0 = i1 - 1;
      const double dt = arc_t_[i1] - arc_t_[i0];
      const double u = dt > 1e-12 ? (t - arc_t_[i0]) / dt : 0.0;
      return arc_s_[i0] + u * (arc_s_[i1] - arc_s_[i0]);
    }

    double timeAtArcS(double s)
    {
      Eigen::Vector3d pos;
      double t = 0.0;
      queryAtArcS(s, pos, t);
      return t;
    }

    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // get Bspline paramterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delt = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector3d prev_pt = first_pt;              // previous point
      Eigen::Vector3d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // get Bspline paramterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_;
    double duration_;
    rclcpp::Time start_time_;
    Eigen::Vector3d start_pos_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace ego_planner

#endif