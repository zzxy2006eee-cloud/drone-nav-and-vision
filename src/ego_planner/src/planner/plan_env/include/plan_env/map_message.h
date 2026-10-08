#ifndef PLAN_ENV_MAP_MESSAGE_H
#define PLAN_ENV_MAP_MESSAGE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/time.h>
#include <string>
#include <cmath>

namespace plan_env {

inline int virtualCeilingIndex(double ceiling, double origin_z,
                              double resolution, int size_z) {
  if (!std::isfinite(ceiling) || ceiling <= -0.5 || resolution <= 0 || size_z <= 0)
    return -1;
  const double index = std::floor((ceiling-origin_z)/resolution);
  return index >= 0 && index < size_z ? static_cast<int>(index) : -1;
}

inline bool aboveVirtualCeiling(double height, double ceiling) {
  return ceiling > -0.5 && height >= ceiling;
}

// Re-publication is not a new observation. Keep the acquisition stamp of
// the input that actually produced these voxels, including zero if unready.
inline sensor_msgs::PointCloud2 makeMapMessage(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const ros::Time& observation_stamp, const std::string& frame_id) {
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;
  message.header.stamp = observation_stamp;
  return message;
}

}  // namespace plan_env

#endif
