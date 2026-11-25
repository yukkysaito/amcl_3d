#include "amcl_3d_core/map/map_factory.hpp"

#include <stdexcept>
#include <utility>

#include "amcl_3d_core/map/cuda_distance_field_map.hpp"
#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"

namespace amcl_3d
{
MapParams::Type toMapType(const std::string & name)
{
  if (name == "distance_field") {
    return MapParams::Type::kDistanceField;
  }
  if (name == "kdtree") {
    return MapParams::Type::kKdTree;
  }
  throw std::invalid_argument("map type must be 'distance_field' or 'kdtree': " + name);
}

MapParams::Device toMapDevice(const std::string & name)
{
  if (name == "cpu") {
    return MapParams::Device::kCpu;
  }
  if (name == "cuda") {
    return MapParams::Device::kCuda;
  }
  if (name == "auto") {
    return MapParams::Device::kAuto;
  }
  throw std::invalid_argument("map device must be 'cpu', 'cuda' or 'auto': " + name);
}

void validate(const MapParams & params)
{
  if (params.type == MapParams::Type::kDistanceField && !(params.resolution > 0.0)) {
    throw std::invalid_argument("map resolution must be positive");
  }
  if (params.device == MapParams::Device::kCuda) {
    if (params.type != MapParams::Type::kDistanceField) {
      throw std::invalid_argument("the CUDA map device requires the distance_field map type");
    }
    if (!CudaDistanceFieldMap::available()) {
      throw std::invalid_argument(
        "the CUDA map device needs amcl_3d_core built with -DAMCL3D_USE_CUDA=ON and a CUDA "
        "device");
    }
  }
}

std::shared_ptr<const PointCloudMap> createPointCloudMap(
  const PointCloud::ConstPtr & cloud, const MapParams & requested, double max_distance)
{
  MapParams params = requested;
  if (params.device == MapParams::Device::kAuto) {
    params.device =
      params.type == MapParams::Type::kDistanceField && CudaDistanceFieldMap::available()
        ? MapParams::Device::kCuda
        : MapParams::Device::kCpu;
  }
  validate(params);
  if (!cloud || cloud->empty()) {
    throw std::invalid_argument("the map point cloud is empty");
  }
  if (params.type == MapParams::Type::kKdTree) {
    return std::make_shared<KdTreeMap>(cloud);
  }
  auto field = std::make_shared<DistanceFieldMap>(
    *cloud, DistanceFieldMap::Params{params.resolution, max_distance});
  if (params.device == MapParams::Device::kCuda) {
    return std::make_shared<CudaDistanceFieldMap>(std::move(field));
  }
  return field;
}

}  // namespace amcl_3d
