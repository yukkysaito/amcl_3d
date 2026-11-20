#include "amcl_3d_core/point_cloud_likelihood/ndt_likelihood.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "parallel_for.hpp"

namespace amcl_3d
{
namespace
{
const std::array<Eigen::Vector3i, 7> kNeighborOffsets{
  Eigen::Vector3i(0, 0, 0),  Eigen::Vector3i(1, 0, 0),  Eigen::Vector3i(-1, 0, 0),
  Eigen::Vector3i(0, 1, 0),  Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1),
  Eigen::Vector3i(0, 0, -1),
};
}  // namespace

void NdtLikelihood::validate(const Params & params)
{
  if (!(params.sigma > 0.0) || !(params.max_mahalanobis_distance > 0.0)) {
    throw std::invalid_argument(
      "NdtLikelihood: sigma and max_mahalanobis_distance must be positive");
  }
}

NdtLikelihood::NdtLikelihood(std::shared_ptr<const GaussianVoxelMap> map, const Params & params)
: map_(std::move(map)), params_(params)
{
  if (!map_) {
    throw std::invalid_argument("NdtLikelihood: map is null");
  }
  validate(params_);
  max_squared_distance_ = params_.max_mahalanobis_distance * params_.max_mahalanobis_distance;
  const Eigen::Matrix3d noise = params_.sigma * params_.sigma * Eigen::Matrix3d::Identity();
  const auto & voxels = map_->voxels();
  information_.reserve(voxels.size());
  for (const auto & voxel : voxels) {
    information_.push_back(
      Eigen::LLT<Eigen::Matrix3d>(voxel.covariance + noise).solve(Eigen::Matrix3d::Identity()));
  }

  // A point in cell c is scored by the voxels at c + offset, so voxel v scores the cells
  // v.coordinate - offset.
  const size_t offset_num = params_.search_neighbors ? kNeighborOffsets.size() : 1;
  std::unordered_map<Eigen::Vector3i, std::vector<uint32_t>, GaussianVoxelMap::CoordinateHash>
    lists;
  for (size_t v = 0; v < voxels.size(); ++v) {
    for (size_t n = 0; n < offset_num; ++n) {
      lists[voxels[v].coordinate - kNeighborOffsets[n]].push_back(static_cast<uint32_t>(v));
    }
  }
  candidates_.reserve(lists.size());
  candidate_voxels_.reserve(voxels.size() * offset_num);
  for (const auto & [cell, list] : lists) {
    candidates_.emplace(
      cell, Candidates{
              static_cast<uint32_t>(candidate_voxels_.size()), static_cast<uint32_t>(list.size())});
    candidate_voxels_.insert(candidate_voxels_.end(), list.begin(), list.end());
  }
}

double NdtLikelihood::scorePoint(const Eigen::Vector3d & point) const
{
  double best = max_squared_distance_;
  const auto it = candidates_.find(map_->voxelCoordinate(point));
  if (it == candidates_.end()) {
    return best;
  }
  const auto & voxels = map_->voxels();
  for (uint32_t k = 0; k < it->second.count; ++k) {
    const uint32_t v = candidate_voxels_[it->second.begin + k];
    const Eigen::Vector3d error = point - voxels[v].mean;
    best = std::min(best, error.dot(information_[v] * error));
  }
  return best;
}

Eigen::VectorXd NdtLikelihood::logLikelihood(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const
{
  Eigen::VectorXd values(static_cast<Eigen::Index>(particles.size()));
  parallelFor(values.size(), params_.thread_num, [&](int64_t i) {
    const Pose pose = particles.pose(static_cast<size_t>(i));
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    double sum = 0.0;
    for (const auto & point : points) {
      sum += scorePoint(rotation * point + pose.position);
    }
    values(i) = -0.5 * sum;
  });
  return values;
}

std::string NdtLikelihood::description() const
{
  std::ostringstream out;
  out << "NDT (sigma " << params_.sigma << " m, max Mahalanobis distance "
      << params_.max_mahalanobis_distance << (params_.search_neighbors ? ", 7 voxels" : ", 1 voxel")
      << ") on " << map_->description();
  // approximate: hash nodes and buckets of the lookup table
  const double table_bytes = static_cast<double>(
    information_.size() * sizeof(Eigen::Matrix3d) + candidate_voxels_.size() * sizeof(uint32_t) +
    candidates_.size() * (sizeof(Eigen::Vector3i) + sizeof(Candidates) + 2 * sizeof(void *)) +
    candidates_.bucket_count() * sizeof(void *));
  out << ", lookup table " << candidates_.size() << " cells, " << std::fixed << std::setprecision(1)
      << table_bytes / 1e6 << " MB";
  return out.str();
}

}  // namespace amcl_3d
