#include "amcl_3d_core/map/gaussian_voxel_map.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace amcl_3d
{
namespace
{
/// Grid coordinates of the map are kept within +-kGridLimit (e.g. +-1e6 km at 1 m, enough for UTM),
/// so that neighbor coordinates do not overflow. Queries beyond it (diverged particles, outliers)
/// map to kOutside, which is no voxel and no neighbor of one.
constexpr double kGridLimit = 1 << 30;
const Eigen::Vector3i kOutside = Eigen::Vector3i::Constant(std::numeric_limits<int>::max());

struct Moments
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();
  size_t count = 0;
};
}  // namespace

size_t GaussianVoxelMap::CoordinateHash::operator()(const Eigen::Vector3i & key) const
{
  // large primes (Teschner et al., 2003)
  return static_cast<size_t>(key.x()) * 73856093u ^ static_cast<size_t>(key.y()) * 19349663u ^
         static_cast<size_t>(key.z()) * 83492791u;
}

void GaussianVoxelMap::validate(const Params & params)
{
  if (!(params.resolution > 0.0) || params.min_point_num == 0) {
    throw std::invalid_argument("GaussianVoxelMap: resolution and min_point_num must be positive");
  }
}

GaussianVoxelMap::GaussianVoxelMap(const PointCloud & cloud, const Params & params)
: params_(params)
{
  validate(params_);
  std::unordered_map<Eigen::Vector3i, Moments, CoordinateHash> moments;
  for (const auto & p : cloud) {
    const Eigen::Vector3d point(p.x, p.y, p.z);
    if (!point.allFinite()) {
      continue;
    }
    const Eigen::Vector3i coordinate = voxelCoordinate(point);
    if (coordinate == kOutside) {
      throw std::invalid_argument("GaussianVoxelMap: the map exceeds the grid range");
    }
    Moments & m = moments[coordinate];
    // Accumulate relative to the voxel corner: keeps the covariance accurate in large coordinates.
    const Eigen::Vector3d local = point - coordinate.cast<double>() * params_.resolution;
    m.sum += local;
    m.sum_outer += local * local.transpose();
    ++m.count;
  }
  voxels_.reserve(moments.size());
  for (const auto & [coordinate, m] : moments) {
    if (m.count < params_.min_point_num) {
      continue;
    }
    const double n = static_cast<double>(m.count);
    const Eigen::Vector3d local_mean = m.sum / n;
    Voxel voxel;
    voxel.coordinate = coordinate;
    voxel.mean = local_mean + coordinate.cast<double>() * params_.resolution;
    voxel.covariance = m.sum_outer / n - local_mean * local_mean.transpose();
    voxel.point_num = m.count;
    index_.emplace(coordinate, static_cast<uint32_t>(voxels_.size()));
    voxels_.push_back(voxel);
  }
  if (voxels_.empty()) {
    throw std::invalid_argument("GaussianVoxelMap: no voxel has min_point_num points");
  }
}

Eigen::Vector3i GaussianVoxelMap::voxelCoordinate(const Eigen::Vector3d & point) const
{
  const Eigen::Vector3d scaled = (point / params_.resolution).array().floor();
  if (!(scaled.cwiseAbs().maxCoeff() < kGridLimit)) {  // also catches NaN
    return kOutside;
  }
  return scaled.cast<int>();
}

int64_t GaussianVoxelMap::find(const Eigen::Vector3i & coordinate) const
{
  const auto it = index_.find(coordinate);
  return it == index_.end() ? -1 : static_cast<int64_t>(it->second);
}

std::string GaussianVoxelMap::description() const
{
  char text[128];
  std::snprintf(
    text, sizeof(text), "Gaussian voxels: %.2f m, %zu voxels, %.1f MB", params_.resolution,
    voxels_.size(), static_cast<double>(voxels_.size() * sizeof(Voxel)) / 1e6);
  return text;
}

}  // namespace amcl_3d
