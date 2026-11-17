#pragma once

// Internal interface between CudaDistanceFieldMap (plain C++) and the CUDA code. It uses only POD
// types so that the .cu file needs neither Eigen nor PCL headers. When the library is built
// without CUDA, device_distance_field_stub.cpp implements it.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace amcl_3d
{
namespace cuda
{
/// Geometry of a DistanceFieldMap, with the sparse blocks looked up through a dense grid of block
/// indices over the bounding box of the blocks (-1: no block).
struct FieldLayout
{
  float resolution;    // [m]
  float offset_scale;  // [m] per quantization step of the site offsets
  int block_size;      // voxels per block edge
  int block_min[3];    // block coordinate of lookup[0]
  int block_dims[3];   // lookup grid size, x fastest
};

/// Rigid transform of one particle: row-major rotation, then translation.
struct Transform
{
  float rotation[9];
  float translation[3];
};

/// Number of CUDA devices; 0 without CUDA.
int deviceCount();

class DeviceDistanceField
{
public:
  /// `site_offsets` and `block_lookup` are copied to `device`. Throws std::runtime_error.
  DeviceDistanceField(
    int device, const FieldLayout & layout, const std::vector<int8_t> & site_offsets,
    const std::vector<int32_t> & block_lookup);
  ~DeviceDistanceField();

  /// sums[i] = sum_k min(|T_i * points[k] - site|^2, clip_squared). `points` are x, y, z triples.
  /// Throws std::runtime_error.
  void sumClippedSquaredDistances(
    const std::vector<Transform> & transforms, const std::vector<float> & points,
    float clip_squared, std::vector<double> & sums);

  size_t memoryBytes() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::mutex mutex_;  // the device buffers are reused between calls
};

}  // namespace cuda
}  // namespace amcl_3d
