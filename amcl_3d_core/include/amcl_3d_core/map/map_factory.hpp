#pragma once

#include <memory>
#include <string>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Selects how the likelihood queries the map. The single place that knows every implementation,
/// so that frontends (the ROS node, the tools) only pass the configuration through.
struct MapParams
{
  enum class Type
  {
    kDistanceField,  // precomputed nearest map points: fast, ~resolution accuracy
    kKdTree,         // exact nearest neighbor search: slow
  };
  enum class Device
  {
    kCpu,
    kCuda,  // distance field only; needs a CUDA build and device
    kAuto,  // kCuda for the distance field when CUDA is available, kCpu otherwise
  };

  Type type = Type::kDistanceField;
  Device device = Device::kCpu;
  double resolution = 0.2;  // [m] distance field voxel size
};

/// Parse the configuration names ("distance_field", "kdtree"; "cpu", "cuda", "auto").
/// Throw std::invalid_argument for unknown names.
MapParams::Type toMapType(const std::string & name);
MapParams::Device toMapDevice(const std::string & name);

/// Throws std::invalid_argument for inconsistent parameters (e.g. CUDA with the k-d tree, or CUDA
/// unavailable), checked before the potentially slow build.
void validate(const MapParams & params);

/// Builds the map of `cloud`, accurate up to `max_distance` [m] (the measurement clipping distance).
/// Throws std::invalid_argument for invalid parameters or an empty cloud.
std::shared_ptr<const PointCloudMap> createPointCloudMap(
  const PointCloud::ConstPtr & cloud, const MapParams & params, double max_distance);

}  // namespace amcl_3d
