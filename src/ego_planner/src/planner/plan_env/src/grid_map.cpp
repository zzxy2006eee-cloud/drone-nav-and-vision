#include "plan_env/grid_map.h"
#include "plan_env/map_message.h"
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <cstdio>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("grid_map/obstacles_inflation_z", mp_.obstacles_inflation_z_, 0.1);
  node_.param("grid_map/retain_cloud_obstacles", mp_.retain_cloud_obstacles_, false);
  std::vector<double> free_min,free_max;
  node_.param("/drone_flight_manager/goal_min_xyz",free_min,std::vector<double>{-8.,-8.,.5});
  node_.param("/drone_flight_manager/goal_max_xyz",free_max,std::vector<double>{8.,8.,2.5});
  if(free_min.size()!=3 || free_max.size()!=3)throw std::runtime_error("Invalid navigation free evidence bounds");
  mp_.free_evidence_min_=Eigen::Vector3d(free_min[0],free_min[1],free_min[2]);
  mp_.free_evidence_max_=Eigen::Vector3d(free_max[0],free_max[1],free_max[2]);
  string cloud_memory_start_topic;
  node_.param("grid_map/cloud_memory_start_topic", cloud_memory_start_topic, string(""));
  cloud_memory_started_ = cloud_memory_start_topic.empty();
  if (!cloud_memory_start_topic.empty())
    cloud_memory_start_sub_ = node_.subscribe(cloud_memory_start_topic, 1,
                                            &GridMap::cloudMemoryStartCallback, this);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, 999.0);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node_.getParam("/drone/max_flight_height_m", mp_.virtual_ceil_height_);
  max_height_sub_ = node_.subscribe<std_msgs::Float64>("/drone/max_flight_height", 1,
      [this](const std_msgs::Float64::ConstPtr& message) {
        if (!std::isfinite(message->data) || message->data < .8 || message->data > 2.5) return;
        if (std::abs(mp_.virtual_ceil_height_ - message->data) < 1e-8) return;
        const int old_ceiling = plan_env::virtualCeilingIndex(mp_.virtual_ceil_height_,
            mp_.map_origin_(2), mp_.resolution_, mp_.map_voxel_num_(2));
        // A ceiling is synthetic occupancy, never cloud evidence. Remove the
        // whole previous plane, including remembered cells outside the local
        // window, restoring actual obstacles from their inflation counts.
        if (old_ceiling >= 0)
          for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
            for (int y = 0; y < mp_.map_voxel_num_(1); ++y) {
              const int address = toAddress(Eigen::Vector3i(x, y, old_ceiling));
              dirty_voxels_.insert(address);
              md_.occupancy_buffer_inflate_[address] = md_.cloud_inflate_refs_[address] > 0;
            }
        mp_.virtual_ceil_height_ = message->data;
        addVirtualCeiling();
        md_.last_published_stamp_ = ros::Time(0);
      });

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node_.param("grid_map/require_observed_free", mp_.require_observed_free_, false);
  std::vector<double> sensor_offset;
  node_.param("grid_map/cloud_sensor_xyz_body", sensor_offset, std::vector<double>{0.0,0.0,0.0});
  if (sensor_offset.size() != 3) throw std::runtime_error("cloud_sensor_xyz_body must have 3 entries");
  mp_.cloud_sensor_offset_ = Eigen::Vector3d(sensor_offset[0], sensor_offset[1], sensor_offset[2]);
  std::vector<double> body_half;
  node_.param("grid_map/cloud_body_half_extent", body_half, std::vector<double>{0.0,0.0,0.0});
  if (body_half.size()!=3) throw std::runtime_error("cloud_body_half_extent must have 3 entries");
  mp_.cloud_body_half_extent_ = Eigen::Vector3d(body_half[0],body_half[1],body_half[2]);
  if (!mp_.cloud_body_half_extent_.allFinite() || mp_.cloud_body_half_extent_.minCoeff()<0)
    throw std::runtime_error("Invalid cloud body extent");
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
  md_.cloud_observed_free_ = vector<char>(buffer_size, 0);
  md_.cloud_evidence_.assign(buffer_size, 0);
  md_.cloud_scan_flags_.assign(buffer_size, 0);
  md_.cloud_inflate_refs_.assign(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;
  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, -0.02,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/grid_map/depth", 50));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  string cloud_pose_topic;
  node_.param("grid_map/cloud_body_pose_topic", cloud_pose_topic, string(""));
  if (cloud_pose_topic.empty()) {
    if (mp_.require_observed_free_) throw std::runtime_error("Observed free rays require synchronized cloud pose");
    indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud", 10, &GridMap::cloudCallback, this);
  } else {
    synced_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/grid_map/cloud", 5));
    cloud_pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, cloud_pose_topic, 5));
    cloud_pose_sync_.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>(
        *synced_cloud_sub_, *cloud_pose_sub_, 5));
    cloud_pose_sync_->registerCallback(boost::bind(&GridMap::cloudPoseCallback, this, _1, _2));
  }
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/grid_map/odom", 10, &GridMap::odomCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy", 1, true);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 1, true);
  map_inf_safety_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate_safety", 1, true);

  voxel_delta_pub_ = node_.advertise<plan_env::VoxelUpdate>("/grid_map/voxel_delta",100);
  voxel_snapshot_service_ = node_.advertiseService("/grid_map/get_voxel_snapshot",&GridMap::voxelSnapshotCallback,this);
  observed_free_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/observed_free", 1, true);
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/unknown", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.last_observation_stamp_ = ros::Time(0);
  md_.last_published_stamp_ = ros::Time(0);
  md_.pending_depth_stamp_ = ros::Time(0);
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = Eigen::Vector3i::Zero();
  md_.image_cnt_ = 0;

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  map_mode_pub_ = node_.advertise<std_msgs::String>("/drone/map_mode",1,true);
  map_archive_status_pub_ = node_.advertise<std_msgs::String>("/drone/map_status",1,true);
  archive_state_sub_ = node_.subscribe<mavros_msgs::State>("/mavros/state",1,[this](const mavros_msgs::State::ConstPtr& m){archive_armed_=m->armed;archive_connected_=m->connected;archive_state_time_=ros::WallTime::now();});
  archive_landed_sub_ = node_.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",1,[this](const mavros_msgs::ExtendedState::ConstPtr& m){archive_landed_=m->landed_state;archive_landed_time_=ros::WallTime::now();});
  archive_phase_sub_ = node_.subscribe<std_msgs::String>("/drone/flight_state",1,[this](const std_msgs::String::ConstPtr& m){archive_phase_=m->data;archive_phase_time_=ros::WallTime::now();});
  archive_health_sub_ = node_.subscribe<std_msgs::String>("/drone/flight_health",1,[this](const std_msgs::String::ConstPtr& m){archive_health_=m->data;archive_health_time_=ros::WallTime::now();});
  archive_queue_sub_ = node_.subscribe<nav_msgs::Path>("/drone/goal_queue",1,[this](const nav_msgs::Path::ConstPtr& m){archive_queue_size_=m->poses.size();});
  map_archive_service_ = node_.advertiseService("/drone/map_archive",&GridMap::mapArchiveCallback,this);
  publishArchiveMode("在线建图导航；可保存地图或落地加载预建地图");

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  navigation_cells_.clear();
  dirty_voxels_.clear();voxel_full_pending_=true;voxel_stamp_=ros::Time(0);
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);
  md_.last_observation_stamp_ = ros::Time(0);
  md_.last_published_stamp_ = ros::Time(0);

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
        if(!voxel_full_pending_)dirty_voxels_.insert(toAddress(x,y,z));
        navigation_cells_.erase(toAddress(x,y,z));
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.cloud_observed_free_[toAddress(x, y, z)] = 0;
        md_.cloud_evidence_[toAddress(x, y, z)] = 0;
        md_.cloud_scan_flags_[toAddress(x, y, z)] = 0;
        md_.cloud_inflate_refs_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

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

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v++)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++)
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
      last_camera_r_inv = md_.last_camera_q_.inverse();
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
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

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

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
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
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  addVirtualCeiling();
}

void GridMap::addVirtualCeiling()
{
  const int ceil_id = plan_env::virtualCeilingIndex(mp_.virtual_ceil_height_,
      mp_.map_origin_(2), mp_.resolution_, mp_.map_voxel_num_(2));
  if (ceil_id < 0)
    return;
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        const int address=toAddress(Eigen::Vector3i(x,y,ceil_id));
        if(!md_.occupancy_buffer_inflate_[address])dirty_voxels_.insert(address);
        md_.occupancy_buffer_inflate_[address]=1;
      }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  // One snapshot per incorporated observation. Latched publishers give late
  // subscribers the same snapshot, without refreshing its acquisition time.
  if (md_.last_observation_stamp_.isZero() ||
      md_.last_observation_stamp_ == md_.last_published_stamp_)
    return;
  publishVoxelDelta();
  publishMap();
  publishMapInflate(true);
  md_.last_published_stamp_ = md_.last_observation_stamp_;
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;
  if (md_.pending_depth_stamp_.isZero() ||
      md_.pending_depth_stamp_ <= md_.last_observation_stamp_)
  {
    md_.occ_need_update_ = false;
    return;
  }

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

  md_.last_observation_stamp_ = md_.pending_depth_stamp_;
  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  if (img->header.stamp.isZero() ||
      img->header.stamp <= md_.last_observation_stamp_ ||
      img->header.stamp < md_.pending_depth_stamp_)
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);
  md_.pending_depth_stamp_ = img->header.stamp;

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
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
}
void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

// PX4's local origin can settle during initial external-pose fusion. Do not
// preserve observations from that changing frame into an airborne static map.
// READY is emitted only after the manager validates localization. Latch once;
// later HOLD/READY transitions must not erase obstacles hidden by the lidar.
void GridMap::cloudMemoryStartCallback(const std_msgs::StringConstPtr& state)
{
  if (cloud_memory_started_ || state->data != "READY")
    return;
  resetBuffer();
  md_.has_cloud_ = false;
  md_.last_observation_stamp_ = ros::Time(0);
  md_.last_published_stamp_ = ros::Time(0);
  cloud_memory_started_ = true;
  ROS_INFO("Static cloud memory started after localization became READY");
}

void GridMap::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                 const geometry_msgs::PoseStampedConstPtr& pose)
{
  if (pose->header.frame_id != mp_.frame_id_) return;
  const auto& p = pose->pose.position;
  const auto& q = pose->pose.orientation;
  Eigen::Vector3d position(p.x,p.y,p.z);
  Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
  if (!position.allFinite() || !rotation.coeffs().allFinite() || std::abs(rotation.norm()-1.0) > .025) return;
  md_.camera_pos_ = position;
  md_.camera_q_ = rotation.normalized();
  md_.has_odom_ = true;
  cloudCallback(cloud);
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{

  if (img->header.frame_id != mp_.frame_id_ || img->header.stamp.isZero() ||
      img->header.stamp <= md_.last_observation_stamp_)
    return;

  pcl::PointCloud<pcl::PointXYZI> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (!md_.camera_pos_.allFinite())
    return;

  bool has_finite_point = false;
  for (const auto &point : latest_cloud.points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
    {
      has_finite_point = true;
      break;
    }
  if (!has_finite_point)
    return;

  // Missing returns alone never clear a blind/occluded obstacle. Only rays
  // actually traversing its raw voxel can remove evidence. Non-memory mode
  // rebuilds from this scan; resetting all buffers keeps reference counts exact.
  if (!mp_.retain_cloud_obstacles_ || !cloud_memory_started_) resetBuffer();

  std::vector<int> touched;
  auto observe = [&](const Eigen::Vector3i& id, unsigned char flag) {
    if (!isInMap(id)) return;
    const int adr = toAddress(id);
    // Keep free evidence only in the configurable execution volume. Rays
    // still remove raw occupied cells outside it, preserving map clearing.
    Eigen::Vector3d center;indexToPos(id,center);
    const bool usable=(center.array()>=mp_.free_evidence_min_.array()).all() && (center.array()<=mp_.free_evidence_max_.array()).all();
    if(flag==2 && !usable && !md_.cloud_evidence_[adr] && !md_.cloud_scan_flags_[adr])return;
    if (!md_.cloud_scan_flags_[adr]) touched.push_back(adr);
    md_.cloud_scan_flags_[adr] |= flag;
  };
  // Register every hit first. A hit wins over every other beam's miss in the
  // same scan, and each voxel receives only one evidence update per scan.
  for (const auto& point : latest_cloud.points) {
    Eigen::Vector3d end(point.x, point.y, point.z);
    if (!end.allFinite()) continue;
    Eigen::Vector3i id; posToIndex(end, id);
    if (point.intensity > .5f) observe(id, 1);
  }
  const Eigen::Vector3d origin = md_.camera_pos_ + md_.camera_q_ * mp_.cloud_sensor_offset_;
  RayCaster raycaster;
  for (const auto& point : latest_cloud.points) {
    Eigen::Vector3d end(point.x, point.y, point.z);
    if (!end.allFinite()) continue;
    const double length = (end-origin).norm();
    if (length <= 0 || length > 40.0) continue;
    // Traverse from the sensor; stop at the grid boundary rather than spend
    // time marching 40 m through unmapped sky. A max-range miss has no hit.
    Eigen::Vector3d clipped = end;
    if (!isInMap(end)) {
      const Eigen::Vector3d direction = end-origin;
      double fraction = 1.;
      for(int axis=0;axis<3;++axis) {
        if(direction[axis]>0) fraction=std::min(fraction,(mp_.map_max_boundary_[axis]-1e-5-origin[axis])/direction[axis]);
        else if(direction[axis]<0) fraction=std::min(fraction,(mp_.map_min_boundary_[axis]+1e-5-origin[axis])/direction[axis]);
      }
      if(fraction<=0) continue;
      clipped=origin+fraction*direction;
    }
    if (!raycaster.setInput(origin/mp_.resolution_, clipped/mp_.resolution_)) continue;
    Eigen::Vector3d ray;
    while (raycaster.step(ray)) {
      const Eigen::Vector3d center = (ray+Eigen::Vector3d::Constant(.5))*mp_.resolution_;
      // Preserve a small band around the return, avoiding surface erosion
      // from voxel quantisation. Never extend free rays behind a return.
      Eigen::Vector3i hit_id,cell_id;
      posToIndex(end,hit_id);posToIndex(center,cell_id);
      // Only protect the actual hit voxel, not a 15 cm halo of historical
      // voxels that otherwise can never be cleared near moved surfaces.
      if (point.intensity > .5f && cell_id == hit_id) continue;
      if ((center-origin).norm() < .45) continue;
      Eigen::Vector3i id; posToIndex(center, id);
      observe(id, 2);
    }
  }
  // Physical body volume is known free, including its near-field blind zone.
  Eigen::Vector3d half = md_.camera_q_.toRotationMatrix().cwiseAbs()*mp_.cloud_body_half_extent_;
  Eigen::Vector3i body_min, body_max;
  posToIndex(md_.camera_pos_-half, body_min); posToIndex(md_.camera_pos_+half, body_max);
  boundIndex(body_min); boundIndex(body_max);
  for (int x=body_min.x(); x<=body_max.x(); ++x)
    for (int y=body_min.y(); y<=body_max.y(); ++y)
      for (int z=body_min.z(); z<=body_max.z(); ++z)
        observe(Eigen::Vector3i(x,y,z), 2);

  const int inf_step = ceil(mp_.obstacles_inflation_/mp_.resolution_);
  const int inf_step_z = ceil(mp_.obstacles_inflation_z_/mp_.resolution_);
  for (const int adr : touched) {
    const unsigned char flags = md_.cloud_scan_flags_[adr];
    md_.cloud_scan_flags_[adr] = 0;
    const unsigned char previous = md_.cloud_evidence_[adr];
    // First hit blocks immediately for safety. Repeated scans confirm it;
    // 2-3 independent free scans remove it. Many rays in one scan cannot.
    const unsigned char next = (flags & 1) ? std::min(3, int(previous)+2) :
                              (previous ? previous-1 : 0);
    const bool old_free=md_.cloud_observed_free_[adr];
    md_.cloud_evidence_[adr] = next;
    navigation_cells_.insert(adr);
    if (flags & 1) md_.cloud_observed_free_[adr] = 0;
    else {
      Eigen::Vector3i id(adr/(mp_.map_voxel_num_(1)*mp_.map_voxel_num_(2)),(adr/mp_.map_voxel_num_(2))%mp_.map_voxel_num_(1),adr%mp_.map_voxel_num_(2));
      Eigen::Vector3d center;indexToPos(id,center);
      md_.cloud_observed_free_[adr]=(center.array()>=mp_.free_evidence_min_.array()).all() && (center.array()<=mp_.free_evidence_max_.array()).all();
      if(!next && !md_.cloud_observed_free_[adr] && !md_.cloud_inflate_refs_[adr])navigation_cells_.erase(adr);
    }
    if(old_free!=bool(md_.cloud_observed_free_[adr]))dirty_voxels_.insert(adr);
    if (bool(previous) == bool(next)) continue;
    const int z0 = adr % mp_.map_voxel_num_(2);
    const int y0 = (adr/mp_.map_voxel_num_(2)) % mp_.map_voxel_num_(1);
    const int x0 = adr/(mp_.map_voxel_num_(2)*mp_.map_voxel_num_(1));
    for (int x=std::max(0,x0-inf_step); x<=std::min(mp_.map_voxel_num_(0)-1,x0+inf_step); ++x)
      for (int y=std::max(0,y0-inf_step); y<=std::min(mp_.map_voxel_num_(1)-1,y0+inf_step); ++y)
        for (int z=std::max(0,z0-inf_step_z); z<=std::min(mp_.map_voxel_num_(2)-1,z0+inf_step_z); ++z) {
          const int inflated = toAddress(x,y,z);
          auto& count = md_.cloud_inflate_refs_[inflated];
          if (next) ++count;
          else if (count) --count;
          const bool old_occupied=md_.occupancy_buffer_inflate_[inflated];
          md_.occupancy_buffer_inflate_[inflated] = count > 0;
          if(old_occupied!=bool(md_.occupancy_buffer_inflate_[inflated]))dirty_voxels_.insert(inflated);
          if (count || md_.cloud_observed_free_[inflated]) navigation_cells_.insert(inflated);
          else navigation_cells_.erase(inflated);
        }
  }
  // Reference counts produce exactly the union of CURRENT raw occupied
  // voxels' inflation boxes, updating only boxes affected by raw changes.
  double min_x = md_.camera_pos_(0)-mp_.local_update_range_(0);
  double min_y = md_.camera_pos_(1)-mp_.local_update_range_(1);
  double min_z = md_.camera_pos_(2)-mp_.local_update_range_(2);
  double max_x = md_.camera_pos_(0)+mp_.local_update_range_(0);
  double max_y = md_.camera_pos_(1)+mp_.local_update_range_(1);
  double max_z = md_.camera_pos_(2)+mp_.local_update_range_(2);

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  if (mp_.retain_cloud_obstacles_)
  {
    // Publish all remembered obstacles inside the current local window,
    // including cells not covered by this scan's narrower point bounds.
    posToIndex(md_.camera_pos_ - mp_.local_update_range_, md_.local_bound_min_);
    posToIndex(md_.camera_pos_ + mp_.local_update_range_, md_.local_bound_max_);
  }

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  addVirtualCeiling();
  md_.has_cloud_ = true;
  md_.last_observation_stamp_ = img->header.stamp;
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

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
  sensor_msgs::PointCloud2 cloud_msg =
      plan_env::makeMapMessage(cloud, md_.last_observation_stamp_, mp_.frame_id_);
  map_pub_.publish(cloud_msg);
}

void GridMap::fillVoxelMetadata(plan_env::VoxelUpdate& m) const
{
  m.header.frame_id=mp_.frame_id_;m.header.stamp=voxel_stamp_;
  m.epoch=voxel_epoch_;m.revision=voxel_revision_;
  m.resolution=mp_.resolution_;
  m.origin.x=mp_.map_origin_.x();m.origin.y=mp_.map_origin_.y();m.origin.z=mp_.map_origin_.z();
  for(int i=0;i<3;++i)m.shape[i]=mp_.map_voxel_num_[i];
}

bool GridMap::voxelSnapshotCallback(plan_env::GetVoxelSnapshot::Request&,
                                  plan_env::GetVoxelSnapshot::Response& response)
{
  response.success=!voxel_full_pending_ && !voxel_stamp_.isZero();
  if(!response.success)return true;
  fillVoxelMetadata(response.update);response.update.full=true;response.update.base_revision=0;
  for(size_t id=0;id<published_occupied_.size();++id) {
    if(published_occupied_[id])response.update.occupied_added.push_back(id);
    if(published_free_[id])response.update.free_added.push_back(id);
  }
  return true;
}

void GridMap::publishVoxelDelta()
{
  if(md_.last_observation_stamp_.isZero())return;
  plan_env::VoxelUpdate m;
  const bool full=voxel_full_pending_ || published_occupied_.size()!=md_.occupancy_buffer_inflate_.size();
  if(full) {
    voxel_epoch_=ros::WallTime::now().toNSec();voxel_revision_=0;
    published_occupied_.assign(md_.occupancy_buffer_inflate_.size(),0);
    published_free_.assign(md_.occupancy_buffer_inflate_.size(),0);
  }
  m.base_revision=voxel_revision_;++voxel_revision_;
  voxel_stamp_=md_.last_observation_stamp_;fillVoxelMetadata(m);m.full=full;
  auto append=[&](size_t id) {
    const bool occupied=md_.occupancy_buffer_inflate_[id];
    const bool free=mp_.require_observed_free_ && md_.cloud_observed_free_[id] && !occupied;
    if(occupied!=bool(published_occupied_[id]))(occupied?m.occupied_added:m.occupied_removed).push_back(id);
    if(free!=bool(published_free_[id]))(free?m.free_added:m.free_removed).push_back(id);
    published_occupied_[id]=occupied;published_free_[id]=free;
  };
  if(full)for(size_t id=0;id<published_occupied_.size();++id)append(id);
  else for(const int id:dirty_voxels_)append(id);
  dirty_voxels_.clear();voxel_full_pending_=false;
  // Publish an empty delta too: acquisition freshness is separate from geometry.
  voxel_delta_pub_.publish(m);
}

void GridMap::publishMapInflate(bool all_info)
{

  const bool visual_due=last_visual_stamp_.isZero() ||
    (md_.last_observation_stamp_-last_visual_stamp_).toSec()>=.25;
  const bool publish_visual=map_inf_pub_.getNumSubscribers()>0 && visual_due;
  const bool publish_safety=map_inf_safety_pub_.getNumSubscribers()>0;
  const bool publish_free=mp_.require_observed_free_ && observed_free_pub_.getNumSubscribers()>0;
  if (!publish_visual && !publish_safety && !publish_free)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud, safety_cloud, free_cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  const int ceil_id = plan_env::virtualCeilingIndex(mp_.virtual_ceil_height_,
      mp_.map_origin_(2), mp_.resolution_, mp_.map_voxel_num_(2));
  if (ceil_id >= 0)
    max_cut(2) = std::max(max_cut(2), ceil_id);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (const int address : navigation_cells_) {
    const int z = address % mp_.map_voxel_num_(2);
    const int y = (address/mp_.map_voxel_num_(2)) % mp_.map_voxel_num_(1);
    const int x = address/(mp_.map_voxel_num_(2)*mp_.map_voxel_num_(1));
    const Eigen::Vector3i id(x,y,z);
        if (publish_free && md_.cloud_observed_free_[address] && !md_.occupancy_buffer_inflate_[address]) {
          Eigen::Vector3d free_pos;
          indexToPos(id,free_pos);
          free_cloud.push_back(pcl::PointXYZ(free_pos(0),free_pos(1),free_pos(2)));
        }
        if (md_.occupancy_buffer_inflate_[address] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(id, pos);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        if (publish_safety)
          safety_cloud.push_back(pt);
        if (publish_visual && pos(2) <= mp_.visualization_truncate_height_)
          cloud.push_back(pt);
  }
  // Synthetic ceiling is never saved as raw obstacle evidence.
  if (ceil_id >= 0)
    for (int x=min_cut.x();x<=max_cut.x();++x)
      for (int y=min_cut.y();y<=max_cut.y();++y) {
        const Eigen::Vector3i id(x,y,ceil_id);
        if (navigation_cells_.count(toAddress(id))) continue;
        Eigen::Vector3d p;indexToPos(id,p);
        safety_cloud.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
        if(p.z()<=mp_.visualization_truncate_height_) cloud.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg =
      plan_env::makeMapMessage(cloud, md_.last_observation_stamp_, mp_.frame_id_);
  if (publish_visual) {
    map_inf_pub_.publish(cloud_msg);last_visual_stamp_=md_.last_observation_stamp_;
  }
  if (publish_safety)
  {
    safety_cloud.width = safety_cloud.points.size();
    safety_cloud.height = 1;
    safety_cloud.is_dense = true;
    map_inf_safety_pub_.publish(plan_env::makeMapMessage(
        safety_cloud, md_.last_observation_stamp_, mp_.frame_id_));
  }

  if (publish_free) {
    free_cloud.width=free_cloud.points.size(); free_cloud.height=1; free_cloud.is_dense=true;
    observed_free_pub_.publish(plan_env::makeMapMessage(free_cloud, md_.last_observation_stamp_, mp_.frame_id_));
  }
  // ROS_INFO("pub map");
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg =
      plan_env::makeMapMessage(cloud, md_.last_observation_stamp_, mp_.frame_id_);
  unknown_pub_.publish(cloud_msg);
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

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  if (img->header.stamp.isZero() ||
      img->header.stamp <= md_.last_observation_stamp_ ||
      img->header.stamp < md_.pending_depth_stamp_)
    return;

  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);    
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
  md_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);
  md_.pending_depth_stamp_ = img->header.stamp;

  md_.occ_need_update_ = true;
}

// GridMap

void GridMap::publishArchiveMode(const std::string& detail)
{
  std_msgs::String mode;mode.data=archive_mode_;map_mode_pub_.publish(mode);
  std_msgs::String status;status.data=detail;map_archive_status_pub_.publish(status);
}

void GridMap::rebuildCloudInflation()
{
  std::fill(md_.cloud_inflate_refs_.begin(),md_.cloud_inflate_refs_.end(),0);
  std::fill(md_.occupancy_buffer_inflate_.begin(),md_.occupancy_buffer_inflate_.end(),0);
  navigation_cells_.clear();voxel_full_pending_=true;voxel_stamp_=ros::Time(0);
  const int xy=std::ceil(mp_.obstacles_inflation_/mp_.resolution_);
  const int zz=std::ceil(mp_.obstacles_inflation_z_/mp_.resolution_);
  for(size_t adr=0;adr<md_.cloud_evidence_.size();++adr) {
    if(md_.cloud_observed_free_[adr]) navigation_cells_.insert(adr);
    if(!md_.cloud_evidence_[adr]) continue;
    int z0=adr%mp_.map_voxel_num_(2),y0=(adr/mp_.map_voxel_num_(2))%mp_.map_voxel_num_(1);
    int x0=adr/(mp_.map_voxel_num_(2)*mp_.map_voxel_num_(1));
    for(int x=std::max(0,x0-xy);x<=std::min(mp_.map_voxel_num_(0)-1,x0+xy);++x)
      for(int y=std::max(0,y0-xy);y<=std::min(mp_.map_voxel_num_(1)-1,y0+xy);++y)
        for(int z=std::max(0,z0-zz);z<=std::min(mp_.map_voxel_num_(2)-1,z0+zz);++z) {
          int id=toAddress(x,y,z);++md_.cloud_inflate_refs_[id];md_.occupancy_buffer_inflate_[id]=1;
          navigation_cells_.insert(id);
        }
  }
  addVirtualCeiling();md_.last_published_stamp_=ros::Time(0);
}

bool GridMap::mapArchiveCallback(plan_env::MapArchive::Request& req,plan_env::MapArchive::Response& res)
{
  res.mode=archive_mode_;
  auto fail=[&](const std::string& reason){res.success=false;res.message=reason;return true;};
  const auto wall=ros::WallTime::now();
  auto fresh=[&](const ros::WallTime& t){return !t.isZero() && (wall-t).toSec()<3.;};
  const bool ground=archive_connected_ && !archive_armed_ && archive_landed_==mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && fresh(archive_state_time_) && fresh(archive_landed_time_);
  if(req.action!="save" && req.action!="load" && req.action!="new" && req.action!="online") return fail("未知地图操作");
  if(!ground) return fail("地图保存或切换需要已确认落地且未解锁");
  if(archive_phase_!="READY" || !fresh(archive_health_time_) || archive_health_!="HEALTHY" || archive_queue_size_) return fail("地图操作需要READY、健康定位及空目标队列");
  if(!cloud_memory_started_ || !md_.has_cloud_ || md_.last_observation_stamp_.isZero() || (ros::Time::now()-md_.last_observation_stamp_).toSec()>2.) return fail("雷达地图尚未就绪或数据已过期");
  try {
    if(req.action=="new" || req.action=="online") {
      resetBuffer();archive_mode_="MAPPING";res.success=true;res.mode=archive_mode_;
      res.message="已清除旧地图，开始在线建图；实时导航继续使用扫描地图";
      publishArchiveMode(res.message);return true;
    }
    if(req.path.empty() || req.path.size()>4096) return fail("地图文件路径无效");
    const std::filesystem::path path(req.path);
    if(!path.is_absolute()) return fail("地图文件需要绝对路径");
    if(req.action=="save") {
      struct Record {int address;int evidence;int free;};std::vector<Record> records;
      for(const int id:navigation_cells_) {
        const int e=md_.cloud_evidence_[id];const int f=md_.cloud_observed_free_[id] && !e;
        if(!e && !f) continue;
        records.push_back({id,e,f});if(e)++res.occupied_count;else ++res.free_count;
      }
      if(!res.occupied_count) return fail("没有可保存的原始障碍体素");
      std::filesystem::create_directories(path.parent_path());
      const auto temporary=req.path+".tmp";
      std::ofstream file(temporary,std::ios::trunc);
      file << std::setprecision(17) << (req.use_map_frame ? "DRONE_GRID_V2 " : "DRONE_GRID_V1 ") << mp_.frame_id_ << ' ' << mp_.resolution_ << ' '
           << mp_.map_origin_.x() << ' ' << mp_.map_origin_.y() << ' ' << mp_.map_origin_.z() << ' '
           << mp_.map_voxel_num_.x() << ' ' << mp_.map_voxel_num_.y() << ' ' << mp_.map_voxel_num_.z() << ' ' << records.size();
      if(req.use_map_frame) {
        if(!std::isfinite(req.offset_x)||!std::isfinite(req.offset_y)||!std::isfinite(req.offset_z)||!std::isfinite(req.yaw_deg))throw std::runtime_error("保存地图坐标变换无效");
        file << ' ' << req.offset_x << ' ' << req.offset_y << ' ' << req.offset_z << ' ' << req.yaw_deg;
      }
      file << '\n';
      for(const auto& r:records)file << r.address << ' ' << r.evidence << ' ' << r.free << '\n';
      file.flush();if(!file)throw std::runtime_error("地图写入失败");file.close();
      std::filesystem::rename(temporary,path);
      res.success=true;res.message="地图已保存："+req.path+"；原始占据"+std::to_string(res.occupied_count)+"，已观测自由"+std::to_string(res.free_count);
      publishArchiveMode(res.message);return true;
    }
    if(!std::isfinite(req.offset_x)||!std::isfinite(req.offset_y)||!std::isfinite(req.offset_z)||!std::isfinite(req.yaw_deg))return fail("地图ENU变换无效");
    if(std::abs(req.offset_x)>30. || std::abs(req.offset_y)>30. || std::abs(req.offset_z)>5. || std::abs(req.yaw_deg)>180.)return fail("地图变换超过允许范围");
    if(std::filesystem::file_size(path)>256*1024*1024) return fail("地图文件超出大小限制");
    std::ifstream file(path);std::string magic,frame;double resolution,ox,oy,oz;int nx,ny,nz;size_t count;
    if(!(file>>magic>>frame>>resolution>>ox>>oy>>oz>>nx>>ny>>nz>>count) || (magic!="DRONE_GRID_V1" && magic!="DRONE_GRID_V2") || frame!=mp_.frame_id_ || !std::isfinite(resolution)||!std::isfinite(ox)||!std::isfinite(oy)||!std::isfinite(oz) || std::abs(resolution-mp_.resolution_)>1e-9 || (Eigen::Vector3d(ox,oy,oz)-mp_.map_origin_).norm()>1e-8 || nx!=mp_.map_voxel_num_.x() || ny!=mp_.map_voxel_num_.y() || nz!=mp_.map_voxel_num_.z() || count>md_.cloud_evidence_.size())return fail("地图版本、坐标系、分辨率或网格尺寸不兼容");
    struct Record {int address,evidence,free;};std::vector<Record> records;records.reserve(count);
    std::unordered_set<int> source_addresses;
    double saved_x=0,saved_y=0,saved_z=0,saved_yaw=0;
    if(magic=="DRONE_GRID_V2" && (!(file>>saved_x>>saved_y>>saved_z>>saved_yaw) || !std::isfinite(saved_x)||!std::isfinite(saved_y)||!std::isfinite(saved_z)||!std::isfinite(saved_yaw)))return fail("地图坐标元数据无效");
    const double yaw=(req.yaw_deg+(req.use_map_frame?saved_yaw:0.))*std::acos(-1.)/180.;
    Eigen::Matrix3d rotation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d offset(req.offset_x,req.offset_y,req.offset_z);
    if(req.use_map_frame)offset+=Eigen::AngleAxisd(req.yaw_deg*std::acos(-1.)/180.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*Eigen::Vector3d(saved_x,saved_y,saved_z);
    // Rotated/partially shifted free voxels do not prove an entire destination
    // voxel free. Import free evidence only for zero yaw and grid-aligned shifts.
    const bool free_aligned=std::abs(std::remainder(yaw,2*std::acos(-1.)))<1e-8 &&
      ((offset/mp_.resolution_).array()-(offset/mp_.resolution_).array().round()).matrix().norm()<1e-8;
    for(size_t n=0;n<count;++n) {
      int id,e,f;if(!(file>>id>>e>>f)||id<0||size_t(id)>=md_.cloud_evidence_.size()||e<0||e>3||f<0||f>1||(!e&&!f)||(e&&f)||!source_addresses.insert(id).second)return fail("地图体素数据无效或重复");
      Eigen::Vector3i old(id/(ny*nz),(id/nz)%ny,id%nz),dest;Eigen::Vector3d position;indexToPos(old,position);position=rotation*position+offset;posToIndex(position,dest);
      if(!isInMap(dest) && !req.clip_to_navigation_grid)return fail("地图对齐后有体素超出网格，加载已拒绝");
      if(e && !free_aligned) {
        // Rasterize the transformed source voxel box conservatively. A rotated
        // voxel is not just a point: preserve every possibly occupied cell.
        const Eigen::Vector3d half=rotation.cwiseAbs()*Eigen::Vector3d::Constant(mp_.resolution_*.5);
        Eigen::Vector3i low,high;posToIndex(position-half+Eigen::Vector3d::Constant(1e-9),low);posToIndex(position+half-Eigen::Vector3d::Constant(1e-9),high);
        // A valid boundary voxel can extend below the navigation grid after
        // a small vertical origin change. Clip only its below-floor portion;
        // its centre must still be in the grid (checked above). Keep rejecting
        // horizontal/top overflow so an invalid alignment cannot lose walls.
        if(req.clip_to_navigation_grid) {
          // Simulation spawn may move the local grid relative to the prior.
          // Intersect voxel boxes with this grid; the flight volume is strictly
          // contained inside it, so no navigable-space obstacle is discarded.
          low=low.cwiseMax(Eigen::Vector3i::Zero());
          high=high.cwiseMin(mp_.map_voxel_num_-Eigen::Vector3i::Ones());
          if((low.array()>high.array()).any())continue;
        }
        low.z()=std::max(0,low.z());
        if(!isInMap(low)||!isInMap(high))return fail("旋转地图边界超出网格");
        for(int x=low.x();x<=high.x();++x)for(int y=low.y();y<=high.y();++y)for(int z=low.z();z<=high.z();++z)
          records.push_back({toAddress(x,y,z),e,0});
      } else if(isInMap(dest))records.push_back({toAddress(dest),e,free_aligned?f:0});
    }
    std::string trailing;if(file>>trailing)return fail("地图包含多余数据");
    if(records.empty())return fail("地图为空");
    bool has_obstacle=false;for(const auto& r:records)has_obstacle|=r.evidence>0;
    if(!has_obstacle)return fail("地图没有原始障碍体素");
    // Validation is complete before changing any live map buffers.
    resetBuffer();
    for(const auto& r:records) {
      md_.cloud_evidence_[r.address]=std::max(int(md_.cloud_evidence_[r.address]),r.evidence);
      if(r.free)md_.cloud_observed_free_[r.address]=1;
    }
    for(size_t id=0;id<md_.cloud_evidence_.size();++id) {
      if(md_.cloud_evidence_[id]){md_.cloud_observed_free_[id]=0;++res.occupied_count;}
      else if(md_.cloud_observed_free_[id])++res.free_count;
    }
    rebuildCloudInflation();archive_mode_="PRIOR_NAV";res.mode=archive_mode_;res.success=true;
    res.message="已加载预建地图导航："+req.path+"；实时雷达继续确认/清除占据";
    if(req.clip_to_navigation_grid)res.message+="；仅加载当前导航网格覆盖部分";
    if(!free_aligned)res.message+="；非网格对齐变换，历史自由空间未导入，等待实时观测";
    publishArchiveMode(res.message);return true;
  }catch(const std::exception& e){return fail(std::string("地图操作失败：")+e.what());}
}
