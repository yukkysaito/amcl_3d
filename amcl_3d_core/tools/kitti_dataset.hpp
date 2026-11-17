// Reader of the KITTI odometry dataset (velodyne scans, calibration, ground truth poses).
#pragma once

#include <string>
#include <vector>

#include "amcl_3d_core/types.hpp"

namespace kitti
{
struct Sequence
{
  std::vector<double> times;
  /// Velodyne poses in a z-up world frame (the first velodyne frame).
  std::vector<amcl_3d::Pose> poses;
  std::string velodyne_dir;
};

/// Reads sequences/<sequence>/{calib.txt,times.txt} and poses/<sequence>.txt under
/// `dataset_dir`. The ground truth camera poses are converted to velodyne poses. Throws
/// std::runtime_error for inconsistent files.
Sequence loadSequence(const std::string & dataset_dir, const std::string & sequence);

/// Points of velodyne/<frame>.bin in the velodyne frame. Throws std::runtime_error if missing.
amcl_3d::PointCloud loadScan(const Sequence & sequence, size_t frame);

}  // namespace kitti
