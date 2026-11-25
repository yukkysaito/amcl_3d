#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace amcl_3d
{
PointCloudMeasurementModel::PointCloudMeasurementModel(
  std::shared_ptr<const PointCloudLikelihood> likelihood, std::vector<Eigen::Vector3d> points)
: likelihood_(std::move(likelihood)), points_(std::move(points)), observation_num_(points_.size())
{
  if (!likelihood_) {
    throw std::invalid_argument("PointCloudMeasurementModel: likelihood is null");
  }
}

LogLikelihood PointCloudMeasurementModel::computeLogLikelihood(const ParticleSet & particles) const
{
  LogLikelihood result;
  result.values = scale_ * likelihood_->logLikelihood(particles, points_);
  result.observation_num = observation_num_;
  return result;
}

bool PointCloudMeasurementModel::computeDerivatives(
  const ParticleSet & particles, LogLikelihoodDerivatives & derivatives) const
{
  if (!likelihood_->logLikelihoodDerivatives(particles, points_, derivatives)) {
    return false;
  }
  if (scale_ != 1.0) {
    derivatives.values *= scale_;
    for (auto & g : derivatives.gradients) {
      g *= scale_;
    }
    for (auto & h : derivatives.hessians) {
      h *= scale_;
    }
  }
  derivatives.observation_num = observation_num_;
  return true;
}

std::unique_ptr<MeasurementModel> PointCloudMeasurementModel::coarse(double fraction) const
{
  const auto stride = static_cast<size_t>(std::lround(1.0 / fraction));
  if (!(fraction > 0.0) || stride <= 1 || points_.size() < 2 * stride) {
    return nullptr;
  }
  std::vector<Eigen::Vector3d> subset;
  for (size_t k = 0; k < points_.size(); k += stride) {
    subset.push_back(points_[k]);
  }
  auto model = std::make_unique<PointCloudMeasurementModel>(likelihood_, std::move(subset));
  model->scale_ =
    scale_ * static_cast<double>(points_.size()) / static_cast<double>(model->points_.size());
  model->observation_num_ = observation_num_;
  return model;
}

}  // namespace amcl_3d
