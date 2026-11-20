#pragma once

#include <memory>
#include <vector>

#include "amcl_3d_core/map/distance_field_map.hpp"

namespace amcl_3d
{
namespace cuda
{
class DeviceDistanceField;
}  // namespace cuda

/// DistanceFieldMap whose batched likelihood query runs on a CUDA GPU.
///
/// The field is uploaded once at construction; each sumClippedSquaredDistances() call uploads the
/// particle poses and scan points, evaluates one CUDA block per particle and downloads one sum per
/// particle. The GPU computes in single precision, so sums differ from the CPU ones by a relative
/// ~1e-6, plus rare points that fall on the other side of a voxel boundary.
/// Single queries (squaredDistanceToNearest) are answered by the CPU field.
///
/// Available only when amcl_3d_core is built with -DAMCL3D_USE_CUDA=ON; check available() before
/// constructing. Calls are serialized internally, so the map may be shared between threads.
class CudaDistanceFieldMap : public PointCloudMap
{
public:
  /// Throws std::runtime_error if CUDA is not available or the upload fails.
  explicit CudaDistanceFieldMap(std::shared_ptr<const DistanceFieldMap> field, int device = 0);
  ~CudaDistanceFieldMap() override;

  /// True if the library was built with CUDA and a CUDA device is present.
  static bool available();

  double squaredDistanceToNearest(const Eigen::Vector3d & point) const override
  {
    return field_->squaredDistanceToNearest(point);
  }
  double maxDistance() const override { return field_->maxDistance(); }

  /// Runs on the GPU (`thread_num` is ignored). Throws std::runtime_error on a CUDA error.
  Eigen::VectorXd sumClippedSquaredDistances(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points,
    double clip_distance, size_t thread_num) const override;
  std::string description() const override;

  const DistanceFieldMap & field() const { return *field_; }
  size_t deviceMemoryBytes() const;

private:
  std::shared_ptr<const DistanceFieldMap> field_;
  std::unique_ptr<cuda::DeviceDistanceField> device_field_;
};

}  // namespace amcl_3d
