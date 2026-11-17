#include "amcl_3d_core/scan_filter.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace amcl_3d
{
namespace
{
struct Centroid
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;
};

uint64_t voxelKey(const Eigen::Vector3d & point, double voxel_size)
{
  // 21 bits per axis: +-1M voxels, far beyond any LiDAR range
  constexpr int64_t kOffset = 1 << 20;
  constexpr uint64_t kMask = (1u << 21) - 1u;
  auto index = [&](double value) {
    return static_cast<uint64_t>(static_cast<int64_t>(std::floor(value / voxel_size)) + kOffset) &
           kMask;
  };
  return index(point.x()) | (index(point.y()) << 21) | (index(point.z()) << 42);
}
}  // namespace

std::vector<Eigen::Vector3d> ScanFilter::apply(const PointCloud & scan) const
{
  const double min_squared_range = params_.min_range * params_.min_range;
  const double max_squared_range = params_.max_range * params_.max_range;
  std::vector<Eigen::Vector3d> points;
  points.reserve(scan.size());
  for (const auto & p : scan) {
    const Eigen::Vector3d point(p.x, p.y, p.z);
    const double squared_range = point.squaredNorm();
    if (
      point.allFinite() && min_squared_range <= squared_range &&
      squared_range <= max_squared_range && params_.min_z <= point.z() &&
      point.z() <= params_.max_z) {
      points.push_back(point);
    }
  }
  if (params_.voxel_size <= 0.0) {
    return points;
  }

  std::unordered_map<uint64_t, Centroid> voxels;
  voxels.reserve(points.size());
  for (const auto & point : points) {
    Centroid & centroid = voxels[voxelKey(point, params_.voxel_size)];
    centroid.sum += point;
    ++centroid.count;
  }
  std::vector<Eigen::Vector3d> downsampled;
  downsampled.reserve(voxels.size());
  for (const auto & entry : voxels) {
    downsampled.push_back(entry.second.sum / entry.second.count);
  }
  return downsampled;
}

}  // namespace amcl_3d
