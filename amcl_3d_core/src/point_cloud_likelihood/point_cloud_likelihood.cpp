#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood.hpp"

#include <utility>

#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"

namespace amcl_3d
{
std::unique_ptr<MeasurementModel> PointCloudLikelihood::createMeasurementModel(
  const std::vector<Eigen::Vector3d> & /*scan*/, std::vector<Eigen::Vector3d> sampled) const
{
  return std::make_unique<PointCloudMeasurementModel>(shared_from_this(), std::move(sampled));
}

}  // namespace amcl_3d
