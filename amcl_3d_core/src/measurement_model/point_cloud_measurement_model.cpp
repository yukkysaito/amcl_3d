#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"

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
  const double inv_two_sigma2 = 1.0 / (2.0 * params_.sigma * params_.sigma);
  LogLikelihood result;
  result.observation_num = points_.size();
  result.values = -inv_two_sigma2 * map_->sumClippedSquaredDistances(
                                      particles, points_, params_.max_distance, params_.thread_num);
  return result;
}

}  // namespace amcl_3d
