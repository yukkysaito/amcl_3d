#include "amcl_3d_core/point_cloud_likelihood/ndt_likelihood.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "parallel_for.hpp"

namespace amcl_3d
{
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
}

double NdtLikelihood::scorePoint(const Eigen::Vector3d & point) const
{
  double best = max_squared_distance_;
  const auto & voxels = map_->voxels();
  for (const uint32_t v : map_->candidates(point)) {
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
      << params_.max_mahalanobis_distance << ") on " << map_->description();
  return out.str();
}

}  // namespace amcl_3d
