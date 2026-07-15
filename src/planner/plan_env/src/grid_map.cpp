#include "plan_env/grid_map.h"

#include <algorithm>
#include <cstdint>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/fx", -1.0);
  node_->declare_parameter("grid_map/fy", -1.0);
  node_->declare_parameter("grid_map/cx", -1.0);
  node_->declare_parameter("grid_map/cy", -1.0);
  node_->declare_parameter("grid_map/use_depth_filter", true);
  node_->declare_parameter("grid_map/depth_filter_tolerance", -1.0);
  node_->declare_parameter("grid_map/depth_filter_maxdist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_mindist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_margin", -1);
  node_->declare_parameter("grid_map/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("grid_map/skip_pixel", -1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/min_ray_length", -0.1);
  node_->declare_parameter("grid_map/max_ray_length", -0.1);
  node_->declare_parameter("grid_map/visualization_truncate_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/frame_id", "global");
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/ground_height", 1.0);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);
  node_->declare_parameter("grid_map/occ_confirm_frames", 3);
  node_->declare_parameter("grid_map/occ_clear_frames", 5);
  node_->declare_parameter("grid_map/use_fixed_publish_window", true);
  node_->declare_parameter("grid_map/map_vis_rate", 25.0);
  node_->declare_parameter("grid_map/ground_filter_enable", true);
  node_->declare_parameter("grid_map/ground_filter_margin", 0.12);
  node_->declare_parameter("grid_map/inflate_xy_only", true);
  node_->declare_parameter("grid_map/column_collision_enable", false);
  node_->declare_parameter("grid_map/column_collision_z_eps", 0.05);
  node_->declare_parameter("grid_map/robot_footprint_enable", true);
  node_->declare_parameter("grid_map/robot_footprint_radius", 0.0);
  node_->declare_parameter("grid_map/robot_footprint_front", 0.10);
  node_->declare_parameter("grid_map/robot_footprint_back", 0.20);
  node_->declare_parameter("grid_map/robot_footprint_left", 0.20);
  node_->declare_parameter("grid_map/robot_footprint_right", 0.20);
  node_->declare_parameter("grid_map/robot_footprint_no_inflate", true);
  node_->declare_parameter("grid_map/robot_footprint_clear_margin", 0.20);

  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x", x_size);
  node_->get_parameter("grid_map/map_size_y", y_size);
  node_->get_parameter("grid_map/map_size_z", z_size);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/fx", mp_.fx_);
  node_->get_parameter("grid_map/fy", mp_.fy_);
  node_->get_parameter("grid_map/cx", mp_.cx_);
  node_->get_parameter("grid_map/cy", mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter", mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin", mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel", mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length", mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length", mp_.max_ray_length_);
  node_->get_parameter("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_);
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);
  node_->get_parameter("grid_map/occ_confirm_frames", mp_.occ_confirm_frames_);
  node_->get_parameter("grid_map/occ_clear_frames", mp_.occ_clear_frames_);
  node_->get_parameter("grid_map/use_fixed_publish_window", mp_.use_fixed_publish_window_);
  node_->get_parameter("grid_map/map_vis_rate", mp_.map_vis_rate_);
  node_->get_parameter("grid_map/ground_filter_enable", mp_.ground_filter_enable_);
  node_->get_parameter("grid_map/ground_filter_margin", mp_.ground_filter_margin_);
  node_->get_parameter("grid_map/inflate_xy_only", mp_.inflate_xy_only_);
  node_->get_parameter("grid_map/column_collision_enable", mp_.column_collision_enable_);
  node_->get_parameter("grid_map/column_collision_z_eps", mp_.column_collision_z_eps_);
  node_->get_parameter("grid_map/robot_footprint_enable", mp_.robot_footprint_enable_);
  node_->get_parameter("grid_map/robot_footprint_radius", mp_.robot_footprint_radius_);
  node_->get_parameter("grid_map/robot_footprint_front", mp_.robot_footprint_front_);
  node_->get_parameter("grid_map/robot_footprint_back", mp_.robot_footprint_back_);
  node_->get_parameter("grid_map/robot_footprint_left", mp_.robot_footprint_left_);
  node_->get_parameter("grid_map/robot_footprint_right", mp_.robot_footprint_right_);
  node_->get_parameter("grid_map/robot_footprint_no_inflate", mp_.robot_footprint_no_inflate_);
  node_->get_parameter("grid_map/robot_footprint_clear_margin", mp_.robot_footprint_clear_margin_);
  mp_.robot_footprint_front_ = std::max(0.0, mp_.robot_footprint_front_);
  mp_.robot_footprint_back_ = std::max(0.0, mp_.robot_footprint_back_);
  mp_.robot_footprint_left_ = std::max(0.0, mp_.robot_footprint_left_);
  mp_.robot_footprint_right_ = std::max(0.0, mp_.robot_footprint_right_);
  mp_.robot_footprint_radius_ = std::max(0.0, mp_.robot_footprint_radius_);
  mp_.robot_footprint_clear_margin_ = std::max(0.0, mp_.robot_footprint_clear_margin_);

  mp_.occ_confirm_frames_ = std::max(1, mp_.occ_confirm_frames_);
  mp_.occ_clear_frames_ = std::max(1, mp_.occ_clear_frames_);
  mp_.map_vis_rate_ = std::max(0.5, mp_.map_vis_rate_);

  mp_.column_collision_z_eps_ = std::max(0.0, mp_.column_collision_z_eps_);

  RCLCPP_INFO(
      node_->get_logger(),
      "[grid_map] footprint enable=%d box F/B/L/R=(%.2f,%.2f,%.2f,%.2f) legacy_r=%.2f "
      "no_inflate=%d clear_margin=%.2f column_collision=%d z_eps=%.3f ground_height=%.3f",
      mp_.robot_footprint_enable_ ? 1 : 0,
      mp_.robot_footprint_front_, mp_.robot_footprint_back_,
      mp_.robot_footprint_left_, mp_.robot_footprint_right_,
      mp_.robot_footprint_radius_,
      mp_.robot_footprint_no_inflate_ ? 1 : 0,
      mp_.robot_footprint_clear_margin_,
      mp_.column_collision_enable_ ? 1 : 0,
      mp_.column_collision_z_eps_,
      mp_.ground_height_);

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.hit_streak_ = vector<uint8_t>(buffer_size, 0);
  md_.miss_streak_ = vector<uint8_t>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  // 初始化 message_filters::Subscriber
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_, "grid_map/depth", rclcpp::QoS(50).get_rmw_qos_profile());

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
        node_, "grid_map/pose", rclcpp::QoS(25).get_rmw_qos_profile());

    sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        node_, "grid_map/odom", rclcpp::QoS(100).get_rmw_qos_profile());

    sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
    sync_image_odom_->registerCallback(
        std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // 定时器
  occ_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.05),
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0 / mp_.map_vis_rate_),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;
  md_.last_inflate_camera_pos_ = Eigen::Vector3d::Constant(1e6);
  md_.occ_changed_indices_.clear();
  md_.robot_pos_.setZero();
  md_.robot_fwd_x_ = 1.0;
  md_.robot_fwd_y_ = 0.0;
  md_.robot_right_x_ = 0.0;
  md_.robot_right_y_ = 1.0;
  md_.has_robot_yaw_ = false;
  md_.has_robot_pos_ = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  if (occ == 1 && isGroundFilteredPoint(pos))
    occ = 0;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  md_.occ_changed_indices_.clear();

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    const bool frame_hit =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns];
    const bool was_occupied = md_.occupancy_buffer_[idx_ctns] > mp_.min_occupancy_log_;

    if (frame_hit)
    {
      md_.hit_streak_[idx_ctns] = std::min<uint8_t>(md_.hit_streak_[idx_ctns] + 1, 255);
      md_.miss_streak_[idx_ctns] = 0;
    }
    else
    {
      md_.miss_streak_[idx_ctns] = std::min<uint8_t>(md_.miss_streak_[idx_ctns] + 1, 255);
      md_.hit_streak_[idx_ctns] = 0;
    }

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    bool apply_hit = frame_hit &&
                     (was_occupied || md_.hit_streak_[idx_ctns] >= mp_.occ_confirm_frames_);
    bool apply_miss = !frame_hit &&
                      (!was_occupied || md_.miss_streak_[idx_ctns] >= mp_.occ_clear_frames_);

    if (!apply_hit && !apply_miss)
      continue;

    double log_odds_update = apply_hit ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      if (was_occupied)
        md_.occ_changed_indices_.push_back(idx_ctns);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      if (was_occupied)
        md_.occ_changed_indices_.push_back(idx_ctns);
      continue;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);

    const bool now_occupied = md_.occupancy_buffer_[idx_ctns] > mp_.min_occupancy_log_;
    if (was_occupied != now_occupied)
      md_.occ_changed_indices_.push_back(idx_ctns);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

bool GridMap::isGroundFilteredPoint(const Eigen::Vector3d &pos) const
{
  if (!mp_.ground_filter_enable_)
    return false;
  return pos(2) < mp_.ground_height_ + mp_.ground_filter_margin_;
}

bool GridMap::isGroundFilteredIndex(const Eigen::Vector3i &id) const
{
  if (!mp_.ground_filter_enable_)
    return false;
  const double z = (id(2) + 0.5) * mp_.resolution_ + mp_.map_origin_(2);
  return z < mp_.ground_height_ + mp_.ground_filter_margin_;
}

int GridMap::inflationKernelSize(int inf_step) const
{
  const int side = 2 * inf_step + 1;
  return mp_.inflate_xy_only_ ? side * side : side * side * side;
}

bool GridMap::robotFootprintActive() const
{
  if (!mp_.robot_footprint_enable_ || !md_.has_robot_pos_)
    return false;
  const bool box =
      mp_.robot_footprint_front_ > 1e-3 || mp_.robot_footprint_back_ > 1e-3 ||
      mp_.robot_footprint_left_ > 1e-3 || mp_.robot_footprint_right_ > 1e-3;
  return box || mp_.robot_footprint_radius_ > 1e-3;
}

void GridMap::setRobotOrientationFromQuat(const Eigen::Quaterniond &q)
{
  const Eigen::Quaterniond qn = q.normalized();
  // OpenVINS / D435 optical: body +Z forward, +X right (match d1_planner_bridge).
  const Eigen::Vector3d fwd = qn * Eigen::Vector3d(0.0, 0.0, 1.0);
  const Eigen::Vector3d right = qn * Eigen::Vector3d(1.0, 0.0, 0.0);
  const double f_xy = std::hypot(fwd.x(), fwd.y());
  const double r_xy = std::hypot(right.x(), right.y());
  if (f_xy > 1e-6)
  {
    md_.robot_fwd_x_ = fwd.x() / f_xy;
    md_.robot_fwd_y_ = fwd.y() / f_xy;
  }
  if (r_xy > 1e-6)
  {
    md_.robot_right_x_ = right.x() / r_xy;
    md_.robot_right_y_ = right.y() / r_xy;
  }
  else if (f_xy > 1e-6)
  {
    // Fallback: planar right = rotate forward by -90° (x,y)->(y,-x)
    md_.robot_right_x_ = md_.robot_fwd_y_;
    md_.robot_right_y_ = -md_.robot_fwd_x_;
  }
  md_.has_robot_yaw_ = (f_xy > 1e-6);
}

bool GridMap::isInsideRobotFootprint(const Eigen::Vector3d &pos) const
{
  return isInsideRobotFootprint(pos, 0.0);
}

bool GridMap::isInsideRobotFootprint(const Eigen::Vector3d &pos, double margin) const
{
  if (!robotFootprintActive())
    return false;

  margin = std::max(0.0, margin);
  const double front = mp_.robot_footprint_front_ + margin;
  const double back = mp_.robot_footprint_back_ + margin;
  const double left = mp_.robot_footprint_left_ + margin;
  const double right = mp_.robot_footprint_right_ + margin;

  const double dx = pos(0) - md_.robot_pos_(0);
  const double dy = pos(1) - md_.robot_pos_(1);

  const bool box =
      mp_.robot_footprint_front_ > 1e-3 || mp_.robot_footprint_back_ > 1e-3 ||
      mp_.robot_footprint_left_ > 1e-3 || mp_.robot_footprint_right_ > 1e-3;

  if (box && md_.has_robot_yaw_)
  {
    const double f = dx * md_.robot_fwd_x_ + dy * md_.robot_fwd_y_;
    const double r = dx * md_.robot_right_x_ + dy * md_.robot_right_y_;
    return f >= -back && f <= front && r >= -left && r <= right;
  }

  if (mp_.robot_footprint_radius_ > 1e-3)
  {
    const double rad = mp_.robot_footprint_radius_ + margin;
    return dx * dx + dy * dy <= rad * rad;
  }

  // Box without yaw yet: axis-aligned extents around robot_pos.
  if (box)
  {
    const double fmax = std::max(front, back);
    const double rmax = std::max(left, right);
    return std::fabs(dx) <= fmax && std::fabs(dy) <= rmax;
  }
  return false;
}

void GridMap::clearRobotFootprint()
{
  if (!robotFootprintActive())
    return;

  const double margin = mp_.robot_footprint_clear_margin_;
  const bool box =
      mp_.robot_footprint_front_ > 1e-3 || mp_.robot_footprint_back_ > 1e-3 ||
      mp_.robot_footprint_left_ > 1e-3 || mp_.robot_footprint_right_ > 1e-3;

  double clear_r = mp_.robot_footprint_radius_ + margin;
  if (box)
  {
    clear_r = std::max(
        clear_r,
        std::hypot(
            std::max(mp_.robot_footprint_front_, mp_.robot_footprint_back_) + margin,
            std::max(mp_.robot_footprint_left_, mp_.robot_footprint_right_) + margin));
  }
  if (clear_r <= 1e-3)
    return;

  const int r_step = static_cast<int>(ceil(clear_r / mp_.resolution_));
  Eigen::Vector3i center_id;
  posToIndex(md_.robot_pos_, center_id);

  for (int dx = -r_step; dx <= r_step; ++dx)
  {
    for (int dy = -r_step; dy <= r_step; ++dy)
    {
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z)
      {
        const Eigen::Vector3i id(center_id(0) + dx, center_id(1) + dy, z);
        if (!isInMap(id))
          continue;
        Eigen::Vector3d pos;
        indexToPos(id, pos);
        if (!isInsideRobotFootprint(pos, margin))
          continue;
        md_.occupancy_buffer_inflate_[toAddress(id)] = 0;
      }
    }
  }
}

void GridMap::getStableLocalIndexBounds(Eigen::Vector3i &min_id, Eigen::Vector3i &max_id,
                                       bool for_publish)
{
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  if (for_publish || mp_.use_fixed_publish_window_)
  {
    int lmm = for_publish ? mp_.local_map_margin_ : mp_.local_map_margin_ / 2;
    min_id -= Eigen::Vector3i(lmm, lmm, lmm);
    max_id += Eigen::Vector3i(lmm, lmm, lmm);
    boundIndex(min_id);
    boundIndex(max_id);
  }
}

void GridMap::inflateOccupiedVoxel(const Eigen::Vector3i &id, int inf_step,
                                   vector<Eigen::Vector3i> &inf_pts)
{
  if (isGroundFilteredIndex(id))
    return;

  // Robot body (and clear_margin shell): never act as an inflation seed.
  if (mp_.robot_footprint_no_inflate_ && robotFootprintActive())
  {
    Eigen::Vector3d seed_pos;
    indexToPos(id, seed_pos);
    if (isInsideRobotFootprint(seed_pos, mp_.robot_footprint_clear_margin_))
      return;
  }

  inflatePoint(id, inf_step, inf_pts);
  Eigen::Vector3i inf_pt;
  const int buffer_size =
      mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  for (int k = 0; k < static_cast<int>(inf_pts.size()); ++k)
  {
    inf_pt = inf_pts[k];
    if (!isInMap(inf_pt))
      continue;
    const int idx_inf = toAddress(inf_pt);
    if (idx_inf < 0 || idx_inf >= buffer_size)
      continue;
    // Do not write inflate back into the robot body / clear shell.
    if (mp_.robot_footprint_no_inflate_ && robotFootprintActive())
    {
      Eigen::Vector3d inf_pos;
      indexToPos(inf_pt, inf_pos);
      if (isInsideRobotFootprint(inf_pos, mp_.robot_footprint_clear_margin_))
        continue;
    }
    md_.occupancy_buffer_inflate_[idx_inf] = 1;
  }
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  vector<Eigen::Vector3i> inf_pts(inflationKernelSize(inf_step));
  Eigen::Vector3i inf_min, inf_max;
  getStableLocalIndexBounds(inf_min, inf_max, false);

  const bool camera_moved =
      (md_.camera_pos_ - md_.last_inflate_camera_pos_).norm() > mp_.resolution_;

  bool has_deocc = false;
  for (const int idx_ctns : md_.occ_changed_indices_)
  {
    if (md_.occupancy_buffer_[idx_ctns] <= mp_.min_occupancy_log_)
    {
      has_deocc = true;
      break;
    }
  }

  if (!md_.occ_changed_indices_.empty())
  {
    if (camera_moved || has_deocc)
    {
      for (int x = inf_min(0); x <= inf_max(0); ++x)
        for (int y = inf_min(1); y <= inf_max(1); ++y)
          for (int z = inf_min(2); z <= inf_max(2); ++z)
            md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;

      for (int x = inf_min(0); x <= inf_max(0); ++x)
        for (int y = inf_min(1); y <= inf_max(1); ++y)
          for (int z = inf_min(2); z <= inf_max(2); ++z)
          {
            if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
              inflateOccupiedVoxel(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
          }

      md_.last_inflate_camera_pos_ = md_.camera_pos_;
    }
    else
    {
      for (const int idx_ctns : md_.occ_changed_indices_)
      {
        if (md_.occupancy_buffer_[idx_ctns] <= mp_.min_occupancy_log_)
          continue;

        const int nz = mp_.map_voxel_num_(2);
        const int ny = mp_.map_voxel_num_(1);
        Eigen::Vector3i id;
        id(0) = idx_ctns / (ny * nz);
        const int rem = idx_ctns - id(0) * ny * nz;
        id(1) = rem / nz;
        id(2) = rem % nz;
        inflateOccupiedVoxel(id, inf_step, inf_pts);
      }
    }
  }
  else if (camera_moved)
  {
    for (int x = inf_min(0); x <= inf_max(0); ++x)
      for (int y = inf_min(1); y <= inf_max(1); ++y)
        for (int z = inf_min(2); z <= inf_max(2); ++z)
          md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;

    for (int x = inf_min(0); x <= inf_max(0); ++x)
      for (int y = inf_min(1); y <= inf_max(1); ++y)
        for (int z = inf_min(2); z <= inf_max(2); ++z)
        {
          if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
            inflateOccupiedVoxel(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
        }

    md_.last_inflate_camera_pos_ = md_.camera_pos_;
  }

  md_.occ_changed_indices_.clear();

  clearRobotFootprint();

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = inf_min(0); x <= inf_max(0); ++x)
      for (int y = inf_min(1); y <= inf_max(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback()
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback()
{
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = node_->now();

  if (!md_.occ_need_update_)
  {
    if (md_.flag_use_depth_fusion &&
        (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "odom or depth lost! now=%f, last_occ_update_time=%f, odom_depth_timeout=%f",
                   node_->now().seconds(),
                   md_.last_occ_update_time_.seconds(),
                   mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = node_->now();
  md_.flag_depth_odom_timeout_ = false;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const geometry_msgs::msg::PoseStamped::ConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  {
    const Eigen::Quaterniond q(pose->pose.orientation.w, pose->pose.orientation.x,
                               pose->pose.orientation.y, pose->pose.orientation.z);
    md_.camera_r_m_ = q.toRotationMatrix();
    md_.robot_pos_ = md_.camera_pos_;
    md_.has_robot_pos_ = true;
    setRobotOrientationFromQuat(q);
  }
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::updateRobotPosition(const Eigen::Vector3d &pos)
{
  md_.robot_pos_ = pos;
  md_.has_robot_pos_ = true;

  if (mp_.pose_type_ != ODOMETRY || md_.has_first_depth_)
    return;

  md_.camera_pos_ = pos;
  md_.has_odom_ = true;
}

void GridMap::updateRobotPose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q)
{
  updateRobotPosition(pos);
  setRobotOrientationFromQuat(q);
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut, max_cut;
  if (mp_.use_fixed_publish_window_)
    getStableLocalIndexBounds(min_cut, max_cut, true);
  else
  {
    min_cut = md_.local_bound_min_;
    max_cut = md_.local_bound_max_;
    int lmm = mp_.local_map_margin_ / 2;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
    boundIndex(min_cut);
    boundIndex(max_cut);
  }

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count()<= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut, max_cut;
  if (mp_.use_fixed_publish_window_ || all_info)
    getStableLocalIndexBounds(min_cut, max_cut, all_info);
  else
  {
    min_cut = md_.local_bound_min_;
    max_cut = md_.local_bound_max_;
  }

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);

  // RCLCPP_INFO(rclcpp::get_logger("publishMapInflate"), "pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

int GridMap::getColumnInflateOccupancy(const Eigen::Vector3d &pos, bool footprint_exempt)
{
  // XY must lie in map; query z is the column top (planning_z / odom z).
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
    return -1;

  if (footprint_exempt && isInsideRobotFootprint(pos))
    return 0;

  const double z_floor = mp_.ground_height_ + mp_.column_collision_z_eps_;
  const double z_top = std::max(pos(2), z_floor);

  Eigen::Vector3d p_lo(pos(0), pos(1), z_floor);
  Eigen::Vector3d p_hi(pos(0), pos(1), z_top);
  // Clamp into map Z so a slightly out-of-band planning_z still scans the covered column.
  p_lo(2) = std::min(std::max(p_lo(2), mp_.map_min_boundary_(2) + 1e-4),
                     mp_.map_max_boundary_(2) - 1e-4);
  p_hi(2) = std::min(std::max(p_hi(2), mp_.map_min_boundary_(2) + 1e-4),
                     mp_.map_max_boundary_(2) - 1e-4);
  if (p_hi(2) < p_lo(2))
    std::swap(p_hi(2), p_lo(2));

  Eigen::Vector3i id_lo, id_hi, id_xy;
  posToIndex(p_lo, id_lo);
  posToIndex(p_hi, id_hi);
  posToIndex(pos, id_xy);
  boundIndex(id_lo);
  boundIndex(id_hi);
  if (id_xy(0) < 0 || id_xy(0) >= mp_.map_voxel_num_(0) || id_xy(1) < 0 ||
      id_xy(1) >= mp_.map_voxel_num_(1))
    return -1;

  for (int iz = id_lo(2); iz <= id_hi(2); ++iz)
  {
    Eigen::Vector3i id(id_xy(0), id_xy(1), iz);
    if (!isInMap(id))
      continue;
    if (md_.occupancy_buffer_inflate_[toAddress(id)] == 1)
      return 1;
  }
  return 0;
}

bool GridMap::checkSegmentInflateOccupied(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1)
{
  // Column mode: only require XY in map; z may sit on planning_z while obstacles are lower.
  if (mp_.column_collision_enable_)
  {
    const auto xy_in_map = [this](const Eigen::Vector3d &p) {
      return p(0) >= mp_.map_min_boundary_(0) + 1e-4 && p(1) >= mp_.map_min_boundary_(1) + 1e-4 &&
             p(0) <= mp_.map_max_boundary_(0) - 1e-4 && p(1) <= mp_.map_max_boundary_(1) - 1e-4;
    };
    if (!xy_in_map(p0) || !xy_in_map(p1))
      return true;
  }
  else if (!isInMap(p0) || !isInMap(p1))
  {
    return true;
  }

  // Match getInflateOccupancy: footprint radius around robot_pos_ is not treated as occupied.
  const auto pos_inflated_occupied = [this](const Eigen::Vector3d &pos) -> bool {
    const int occ = getInflateOccupancy(pos);
    if (occ < 0)
      return true;
    return occ > 0;
  };

  if (pos_inflated_occupied(p0))
    return true;

  Eigen::Vector3i id0, id1;
  posToIndex(p0, id0);
  posToIndex(p1, id1);

  if (id0 == id1)
    return pos_inflated_occupied(p1);

  RayCaster raycaster;
  if (!raycaster.setInput(id0.cast<double>(), id1.cast<double>()))
    return pos_inflated_occupied(p1);

  Eigen::Vector3d ray_pt;
  Eigen::Vector3d pos_w;
  while (raycaster.step(ray_pt))
  {
    const Eigen::Vector3i id(
      static_cast<int>(ray_pt.x()),
      static_cast<int>(ray_pt.y()),
      static_cast<int>(ray_pt.z()));
    indexToPos(id, pos_w);
    if (pos_inflated_occupied(pos_w))
      return true;
  }

  return pos_inflated_occupied(p1);
}

void GridMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  md_.robot_pos_(0) = odom->pose.pose.position.x;
  md_.robot_pos_(1) = odom->pose.pose.position.y;
  md_.robot_pos_(2) = odom->pose.pose.position.z;
  md_.has_robot_pos_ = true;

  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  setRobotOrientationFromQuat(body_q);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}