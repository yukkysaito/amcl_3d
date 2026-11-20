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

private:
  std::shared_ptr<const PointCloudLikelihood> likelihood_;
  std::vector<Eigen::Vector3d> points_;
};

}  // namespace amcl_3d
