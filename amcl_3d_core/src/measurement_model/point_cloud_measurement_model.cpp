#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"

#include <stdexcept>
#include <utility>

namespace amcl_3d
{
PointCloudMeasurementModel::PointCloudMeasurementModel(
  std::shared_ptr<const PointCloudLikelihood> likelihood, std::vector<Eigen::Vector3d> points)
: likelihood_(std::move(likelihood)), points_(std::move(points))
{
  if (!likelihood_) {
    throw std::invalid_argument("PointCloudMeasurementModel: likelihood is null");
  }
}

LogLikelihood PointCloudMeasurementModel::computeLogLikelihood(const ParticleSet & particles) const
{
  LogLikelihood result;
  result.values = likelihood_->logLikelihood(particles, points_);
  result.observation_num = points_.size();
  return result;
}

}  // namespace amcl_3d
