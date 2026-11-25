#pragma once

#include <memory>
#include <vector>

#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood.hpp"

namespace amcl_3d
{
/// One point cloud measurement evaluated by a PointCloudLikelihood.
class PointCloudMeasurementModel : public MeasurementModel
{
public:
  /// `points` are in the base frame, already filtered and sampled.
  /// Throws std::invalid_argument if `likelihood` is null.
  PointCloudMeasurementModel(
    std::shared_ptr<const PointCloudLikelihood> likelihood, std::vector<Eigen::Vector3d> points);

  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override;
  bool computeDerivatives(
    const ParticleSet & particles, LogLikelihoodDerivatives & derivatives) const override;
  /// Every k-th point (k = round(1 / fraction)), scaled by the full count over the subset's.
  std::unique_ptr<MeasurementModel> coarse(double fraction) const override;

private:
  std::shared_ptr<const PointCloudLikelihood> likelihood_;
  std::vector<Eigen::Vector3d> points_;
  double scale_ = 1.0;          // of the log-likelihood (coarse models)
  size_t observation_num_ = 0;  // reported, of the full model
};

}  // namespace amcl_3d
