#include "kitti_dataset.hpp"

#include <Eigen/Geometry>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace kitti
{
namespace
{
/// 3x4 row-major matrix line -> isometry
Eigen::Isometry3d parseMatrix34(std::istream & stream)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      stream >> m(r, c);
    }
  }
  return Eigen::Isometry3d(m);
}
}  // namespace

Sequence loadSequence(const std::string & dataset_dir, const std::string & sequence)
{
  const std::string sequence_dir = dataset_dir + "/sequences/" + sequence;
  Sequence s;
  s.velodyne_dir = sequence_dir + "/velodyne";

  std::ifstream calib(sequence_dir + "/calib.txt");
  Eigen::Isometry3d camera_from_velodyne = Eigen::Isometry3d::Identity();
  for (std::string line; std::getline(calib, line);) {
    if (line.rfind("Tr:", 0) == 0) {
      std::istringstream stream(line.substr(3));
      camera_from_velodyne = parseMatrix34(stream);
    }
  }

  std::ifstream times(sequence_dir + "/times.txt");
  for (double t; times >> t;) {
    s.times.push_back(t);
  }

  // ground truth: camera poses in the first camera frame (y down, z forward)
  std::ifstream poses(dataset_dir + "/poses/" + sequence + ".txt");
  for (std::string line; std::getline(poses, line);) {
    std::istringstream stream(line);
    const Eigen::Isometry3d world_camera_from_camera = parseMatrix34(stream);
    const Eigen::Isometry3d velodyne_pose =
      camera_from_velodyne.inverse() * world_camera_from_camera * camera_from_velodyne;
    amcl_3d::Pose pose;
    pose.position = velodyne_pose.translation();
    pose.orientation = Eigen::Quaterniond(velodyne_pose.rotation()).normalized();
    s.poses.push_back(pose);
  }
  if (s.times.empty() || s.times.size() != s.poses.size()) {
    throw std::runtime_error("inconsistent times / poses in " + sequence_dir);
  }
  return s;
}

amcl_3d::PointCloud loadScan(const Sequence & s, size_t frame)
{
  char name[16];
  std::snprintf(name, sizeof(name), "%06zu.bin", frame);
  std::ifstream file(s.velodyne_dir + "/" + name, std::ios::binary);
  if (!file) {
    throw std::runtime_error(std::string("missing scan ") + name);
  }
  amcl_3d::PointCloud cloud;
  float point[4];  // x, y, z, reflectance
  while (file.read(reinterpret_cast<char *>(point), sizeof(point))) {
    cloud.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
  }
  return cloud;
}

}  // namespace kitti
