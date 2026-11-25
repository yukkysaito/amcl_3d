#pragma once

#include <memory>
#include <string>

#include "amcl_3d_core/map/gaussian_voxel_map.hpp"
#include "amcl_3d_core/map/map_factory.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
#include "amcl_3d_core/point_cloud_likelihood/ndt_likelihood.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Selects the point cloud likelihood and the map representation it is built on. The single place
/// that knows every implementation, so that frontends (the ROS node, the tools) only pass the
/// configuration through.
struct PointCloudLikelihoodParams
{
  enum class Type
  {
    kLikelihoodField,  // distance to the nearest map point (distance field, k-d tree, CUDA)
    kNdt,              // Mahalanobis distance to the normal distributions of the voxels
  };
  Type type = Type::kLikelihoodField;

  struct LikelihoodFieldParams
  {
    MapParams map;
    LikelihoodField::Params model;
  } likelihood_field;

  struct NdtParams
  {
    GaussianVoxelMap::Params map;
    NdtLikelihood::Params model;
  } ndt;

  /// Sets the CPU threads of every model.
  void setThreadNum(size_t thread_num);
};

/// Parses "likelihood_field" or "ndt". Throws std::invalid_argument for unknown names.
PointCloudLikelihoodParams::Type toPointCloudLikelihoodType(const std::string & name);

/// Throws std::invalid_argument for invalid parameters of the selected type, checked before the
/// potentially slow map build.
void validate(const PointCloudLikelihoodParams & params);

/// Builds the selected likelihood with its map of `cloud`.
/// Throws std::invalid_argument for invalid parameters or an empty cloud.
std::shared_ptr<const PointCloudLikelihood> createPointCloudLikelihood(
  const PointCloud::ConstPtr & cloud, const PointCloudLikelihoodParams & params);

}  // namespace amcl_3d
