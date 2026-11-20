#include "amcl_3d_core/map/cuda_distance_field_map.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "map/cuda/device_distance_field.hpp"

namespace amcl_3d
{
namespace
{
struct DeviceLayout
{
  cuda::FieldLayout layout;
  std::vector<int32_t> lookup;
  Eigen::Vector3d origin;  // world position of the corner of lookup[0]
};

/// Dense grid of block indices over the bounding box of the field's blocks. The device works
/// relative to the corner of that grid, near zero, where single precision stays fine even for maps
/// in projected coordinates (5e5 m is a 0.06 m float step).
DeviceLayout makeLayout(const DistanceFieldMap & field)
{
  const std::vector<Eigen::Vector3i> blocks = field.blockCoordinates();
  Eigen::Vector3i min_block = blocks.front();
  Eigen::Vector3i max_block = blocks.front();
  for (const auto & block : blocks) {
    min_block = min_block.cwiseMin(block);
    max_block = max_block.cwiseMax(block);
  }
  const Eigen::Vector3i dims = max_block - min_block + Eigen::Vector3i::Ones();

  cuda::FieldLayout layout{};
  layout.resolution = static_cast<float>(field.params().resolution);
  layout.offset_scale = static_cast<float>(field.offsetScale());
  layout.block_size = field.blockSize();
  for (int axis = 0; axis < 3; ++axis) {
    layout.block_dims[axis] = dims[axis];
  }
  std::vector<int32_t> lookup(static_cast<size_t>(dims.x()) * dims.y() * dims.z(), -1);
  for (size_t i = 0; i < blocks.size(); ++i) {
    const Eigen::Vector3i g = blocks[i] - min_block;
    lookup[(static_cast<size_t>(g.z()) * dims.y() + g.y()) * dims.x() + g.x()] =
      static_cast<int32_t>(i);
  }
  const Eigen::Vector3d origin =
    min_block.cast<double>() * (field.blockSize() * field.params().resolution);
  return {layout, std::move(lookup), origin};
}
}  // namespace

CudaDistanceFieldMap::CudaDistanceFieldMap(
  std::shared_ptr<const DistanceFieldMap> field, int device)
: field_(std::move(field))
{
  if (!field_) {
    throw std::invalid_argument("CudaDistanceFieldMap: field is null");
  }
  const DeviceLayout layout = makeLayout(*field_);
  origin_ = layout.origin;
  device_field_ = std::make_unique<cuda::DeviceDistanceField>(
    device, layout.layout, field_->siteOffsets(), layout.lookup);
}

CudaDistanceFieldMap::~CudaDistanceFieldMap() = default;

bool CudaDistanceFieldMap::available() { return cuda::deviceCount() > 0; }

Eigen::VectorXd CudaDistanceFieldMap::sumClippedSquaredDistances(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points, double clip_distance,
  size_t /*thread_num*/) const
{
  // DistanceFieldMap clips at its own max_distance, so a larger clip has no effect beyond it.
  const double clip = std::min(clip_distance, field_->maxDistance());

  using RowMajorMatrix3f = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;
  std::vector<cuda::Transform> transforms(particles.size());
  for (size_t i = 0; i < particles.size(); ++i) {
    const Pose pose = particles.pose(i);
    Eigen::Map<RowMajorMatrix3f> rotation(transforms[i].rotation);
    Eigen::Map<Eigen::Vector3f> translation(transforms[i].translation);
    rotation = pose.orientation.toRotationMatrix().cast<float>();
    // the device works relative to origin_: subtract it in double precision
    translation = (pose.position - origin_).cast<float>();
  }
  std::vector<float> flat_points(points.size() * 3);
  for (size_t k = 0; k < points.size(); ++k) {
    Eigen::Map<Eigen::Vector3f> point(&flat_points[k * 3]);
    point = points[k].cast<float>();
  }

  std::vector<double> sums;
  device_field_->sumClippedSquaredDistances(
    transforms, flat_points, static_cast<float>(clip * clip), sums);
  return Eigen::Map<const Eigen::VectorXd>(sums.data(), static_cast<Eigen::Index>(sums.size()));
}

std::string CudaDistanceFieldMap::description() const
{
  char text[64];
  std::snprintf(
    text, sizeof(text), " on the GPU (%.1f MB)", static_cast<double>(deviceMemoryBytes()) / 1e6);
  return field_->description() + text;
}

size_t CudaDistanceFieldMap::deviceMemoryBytes() const { return device_field_->memoryBytes(); }

}  // namespace amcl_3d
