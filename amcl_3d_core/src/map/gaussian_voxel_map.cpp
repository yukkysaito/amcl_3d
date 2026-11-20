#include "amcl_3d_core/map/gaussian_voxel_map.hpp"

#include <array>
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

const std::array<Eigen::Vector3i, 7> kNeighborOffsets{
  Eigen::Vector3i(0, 0, 0),  Eigen::Vector3i(1, 0, 0),  Eigen::Vector3i(-1, 0, 0),
  Eigen::Vector3i(0, 1, 0),  Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1),
  Eigen::Vector3i(0, 0, -1),
};

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
  buildCandidates();
}

void GaussianVoxelMap::buildCandidates()
{
  // A point in cell c is matched with the voxels at c + offset, so voxel v serves the cells
  // v.coordinate - offset.
  const size_t offset_num = params_.search_neighbors ? kNeighborOffsets.size() : 1;
  std::unordered_map<Eigen::Vector3i, std::vector<uint32_t>, CoordinateHash> lists;
  for (size_t v = 0; v < voxels_.size(); ++v) {
    for (size_t n = 0; n < offset_num; ++n) {
      lists[voxels_[v].coordinate - kNeighborOffsets[n]].push_back(static_cast<uint32_t>(v));
    }
  }
  candidates_.reserve(lists.size());
  candidate_voxels_.reserve(voxels_.size() * offset_num);
  for (const auto & [cell, list] : lists) {
    candidates_.emplace(
      cell,
      Span{static_cast<uint32_t>(candidate_voxels_.size()), static_cast<uint32_t>(list.size())});
    candidate_voxels_.insert(candidate_voxels_.end(), list.begin(), list.end());
  }
}

GaussianVoxelMap::CandidateRange GaussianVoxelMap::candidates(const Eigen::Vector3d & point) const
{
  const auto it = candidates_.find(voxelCoordinate(point));
  if (it == candidates_.end()) {
    return CandidateRange(nullptr, nullptr);
  }
  const uint32_t * begin = candidate_voxels_.data() + it->second.begin;
  return CandidateRange(begin, begin + it->second.count);
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
  // approximate: hash tables counted as nodes plus buckets
  const auto hash_bytes = [](const auto & table, size_t entry_bytes) {
    return table.size() * (entry_bytes + 2 * sizeof(void *)) +
           table.bucket_count() * sizeof(void *);
  };
  const size_t bytes = voxels_.size() * sizeof(Voxel) +
                       hash_bytes(index_, sizeof(Eigen::Vector3i) + sizeof(uint32_t)) +
                       hash_bytes(candidates_, sizeof(Eigen::Vector3i) + sizeof(Span)) +
                       candidate_voxels_.size() * sizeof(uint32_t);
  char text[160];
  std::snprintf(
    text, sizeof(text), "Gaussian voxels: %.2f m, %zu voxels, %s, %.1f MB", params_.resolution,
    voxels_.size(), params_.search_neighbors ? "7 voxels per point" : "1 voxel per point",
    static_cast<double>(bytes) / 1e6);
  return text;
}

}  // namespace amcl_3d
