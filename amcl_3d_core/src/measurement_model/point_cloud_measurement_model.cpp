#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace amcl_3d
{
PointCloudMeasurementModel::PointCloudMeasurementModel(
  std::shared_ptr<const PointCloudMap> map, std::vector<Eigen::Vector3d> points,
  const Params & params)
: map_(std::move(map)), points_(std::move(points)), params_(params)
{
  if (!map_) {
    throw std::invalid_argument("PointCloudMeasurementModel: map is null");
  }
  if (params_.sigma <= 0.0) {
    throw std::invalid_argument("PointCloudMeasurementModel: sigma must be positive");
  }
  if (map_->maxDistance() < params_.max_distance) {
    throw std::invalid_argument(
      "PointCloudMeasurementModel: max_distance exceeds the distance range of the map");
  }
}

LogLikelihood PointCloudMeasurementModel::computeLogLikelihood(const ParticleSet & particles) const
{
  const double max_squared_distance = params_.max_distance * params_.max_distance;
  const double inv_two_sigma2 = 1.0 / (2.0 * params_.sigma * params_.sigma);

  LogLikelihood result;
  result.observation_num = points_.size();
  result.values.resize(static_cast<Eigen::Index>(particles.size()));
  // Particles are independent; this loop is the hot spot to parallelize (OpenMP / CUDA).
  for (size_t i = 0; i < particles.size(); ++i) {
    const Pose pose = particles.pose(i);
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    double sum_squared_distance = 0.0;
    for (const auto & point : points_) {
      const Eigen::Vector3d map_point = rotation * point + pose.position;
      sum_squared_distance +=
        std::min(map_->squaredDistanceToNearest(map_point), max_squared_distance);
    }
    result.values(static_cast<Eigen::Index>(i)) = -sum_squared_distance * inv_two_sigma2;
  }
  return result;
}

}  // namespace amcl_3d
