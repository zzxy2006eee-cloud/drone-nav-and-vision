// No ROS master, NodeHandle, simulator or network connection is needed.
#include <plan_env/map_message.h>
#include <ros/serialization.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

void writeMessage(const std::string& path, const sensor_msgs::PointCloud2& message) {
  std::vector<uint8_t> bytes(ros::serialization::serializationLength(message));
  ros::serialization::OStream stream(bytes.data(), bytes.size());
  ros::serialization::serialize(stream, message);
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  require(output.good(), "Cannot write serialized map fixture");
}

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(2.05f, 0.05f, 1.05f));
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "odom";

  sensor_msgs::PointCloud2 legacy;
  pcl::toROSMsg(cloud, legacy);
  require(legacy.header.stamp.isZero(), "Legacy zero-stamp failure not reproduced");

  const ros::Time observation(99, 950123456);
  const auto fresh = plan_env::makeMapMessage(cloud, observation, "odom");
  require(fresh.header.stamp == observation, "Acquisition nanoseconds lost");
  require(fresh.header.frame_id == "odom", "ENU frame lost");
  require(fresh.data == legacy.data, "Voxel data changed while stamping");
  const auto republished = plan_env::makeMapMessage(cloud, observation, "odom");
  require(republished.header.stamp == fresh.header.stamp,
          "Repeated publication changed acquisition time");
  require(plan_env::makeMapMessage(cloud, ros::Time(0), "odom").header.stamp.isZero(),
          "Uninitialized observation advertised as fresh");

  std::vector<uint8_t> bytes(ros::serialization::serializationLength(fresh));
  ros::serialization::OStream output(bytes.data(), bytes.size());
  ros::serialization::serialize(output, fresh);
  ros::serialization::IStream input(bytes.data(), bytes.size());
  sensor_msgs::PointCloud2 restored;
  ros::serialization::deserialize(input, restored);
  require(restored.header.stamp == observation && restored.data == fresh.data,
          "ROS serialization lost time or voxel data");
  require(plan_env::virtualCeilingIndex(2.6, .5, .1, 80) == 21,
          "Ceiling voxel index incorrect");
  require(plan_env::virtualCeilingIndex(-1, .5, .1, 80) == -1,
          "Disabled ceiling has an index");
  require(plan_env::virtualCeilingIndex(.1, .5, .1, 80) == -1,
          "Ceiling below map would address a negative voxel");
  require(plan_env::virtualCeilingIndex(9, .5, .1, 80) == -1,
          "Ceiling outside map would overflow voxel storage");
  require(!plan_env::aboveVirtualCeiling(2.5, 2.6) &&
          plan_env::aboveVirtualCeiling(2.6, 2.6) &&
          plan_env::aboveVirtualCeiling(4, 2.6), "Above-ceiling volume is not blocked");
  require(!plan_env::aboveVirtualCeiling(4, -1), "Disabled ceiling blocks space");
  if (argc == 2) {
    writeMessage(std::string(argv[1]) + "/fresh_map.bin", fresh);
    writeMessage(std::string(argv[1]) + "/legacy_zero_map.bin", legacy);
  }
  std::cout << "PASS: 13 map timestamp/conversion/serialization/ceiling checks\n";
}
