#pragma once

#include <memory>
#include <vector>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/measurement_model/measurement_model.hpp"

namespace amcl_3d
{
/// Likelihood of scan points given the distance to the nearest map point:
///   log p(z | x) = sum_k -min(d_k^2, max_distance^2) / (2 * sigma^2)
/// where d_k is the distance of scan point k, transformed by the particle pose, to the map.
/// Clipping at `max_distance` makes the model robust to points not in the map (dynamic objects).
class PointCloudMeasurementModel : public MeasurementModel
{
public:
  struct Params
  {
    double max_distance = 1.0;  // [m]
    double sigma = 0.2;         // [m]
  };

  /// `points` are in the base frame. The caller selects/downsamples them beforehand.
  PointCloudMeasurementModel(
    std::shared_ptr<const PointCloudMap> map, std::vector<Eigen::Vector3d> points,
    const Params & params);

  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override;

private:
  std::shared_ptr<const PointCloudMap> map_;
  std::vector<Eigen::Vector3d> points_;
  Params params_;
};

}  // namespace amcl_3d
