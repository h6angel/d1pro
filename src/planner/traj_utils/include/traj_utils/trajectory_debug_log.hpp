#ifndef TRAJ_UTILS__TRAJECTORY_DEBUG_LOG_HPP_
#define TRAJ_UTILS__TRAJECTORY_DEBUG_LOG_HPP_

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace traj_utils
{

inline std::string formatXYZ(double x, double y, double z)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << "(" << x << "," << y << "," << z << ")";
  return oss.str();
}

inline std::string formatVec3(const Eigen::Vector3d & p)
{
  return formatXYZ(p.x(), p.y(), p.z());
}

inline std::string formatControlPointsSummary(
  const std::vector<geometry_msgs::msg::Point> & pts, int max_show = 4)
{
  if (pts.empty()) {
    return "[]";
  }
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  const int n = static_cast<int>(pts.size());
  const int show = std::min(max_show, n);
  oss << "n=" << n << " [";
  for (int i = 0; i < show; ++i) {
    if (i > 0) {
      oss << " ";
    }
    oss << "p" << i << "=" << formatXYZ(pts[i].x, pts[i].y, pts[i].z);
  }
  if (n > show) {
    const auto & last = pts.back();
    oss << " ... p" << (n - 1) << "=" << formatXYZ(last.x, last.y, last.z);
  }
  oss << "]";
  return oss.str();
}

inline std::string formatControlPointsSummary(
  const Eigen::MatrixXd & pos_pts, int max_show = 4)
{
  std::vector<geometry_msgs::msg::Point> pts(static_cast<size_t>(pos_pts.cols()));
  for (int i = 0; i < pos_pts.cols(); ++i) {
    pts[static_cast<size_t>(i)].x = pos_pts(0, i);
    pts[static_cast<size_t>(i)].y = pos_pts(1, i);
    pts[static_cast<size_t>(i)].z = pos_pts(2, i);
  }
  return formatControlPointsSummary(pts, max_show);
}

}  // namespace traj_utils

#endif  // TRAJ_UTILS__TRAJECTORY_DEBUG_LOG_HPP_
