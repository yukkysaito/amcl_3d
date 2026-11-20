#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Precomputed nearest map point on a voxel grid (a "likelihood field" in vector form).
/// Queries are O(1) regardless of the map size, and the flat block storage can be uploaded to a
/// GPU as is.
///
/// The map points are summarized by the centroid of each occupied voxel ("site"). Every voxel
/// within `max_distance` of the map stores the offset from its center to the nearest site (found
/// with Felzenszwalb & Huttenlocher's distance transform extended to a feature transform),
/// quantized to 8 bits per axis. A query returns the exact distance from the query point to the
/// site of its voxel, so the field is sharp at surfaces and unbiased; interpolating distances
/// instead would flatten the minimum over a voxel and bias the pose estimate.
/// Typical error is a fraction of the resolution, the worst case about sqrt(3) * resolution.
/// Memory: 3 bytes per voxel within max_distance of the map, in sparse blocks.
class DistanceFieldMap : public PointCloudMap
{
public:
  struct Params
  {
    double resolution = 0.2;    // [m] voxel size
    double max_distance = 1.0;  // [m] distances are clipped here
  };

  /// Throws std::invalid_argument for an empty cloud or non positive parameters.
  DistanceFieldMap(const PointCloud & cloud, const Params & params);

  double squaredDistanceToNearest(const Eigen::Vector3d & point) const override;
  double maxDistance() const override { return params_.max_distance; }
  std::string description() const override;

  size_t blockNum() const { return block_index_.size(); }
  size_t memoryBytes() const { return site_offsets_.size() * sizeof(int8_t); }

  // Raw storage, for backends that copy the field (e.g. CudaDistanceFieldMap).

  /// Marker in the x offset of voxels with no map point within max_distance.
  static constexpr int8_t kNoSite = -128;
  const Params & params() const { return params_; }
  /// Voxels per block edge.
  int blockSize() const { return block_size_; }
  /// [m] per quantization step of the site offsets.
  double offsetScale() const { return offset_scale_; }
  /// Offset (x, y, z) from each voxel center to its site, block after block in the order of
  /// blockCoordinates(), x fastest within a block.
  const std::vector<int8_t> & siteOffsets() const { return site_offsets_; }
  /// Block coordinate (voxel coordinate / blockSize()) of each block in siteOffsets().
  std::vector<Eigen::Vector3i> blockCoordinates() const;

private:
  struct KeyHash
  {
    size_t operator()(const Eigen::Vector3i & key) const;
  };

  void build(const PointCloud & cloud);

  Params params_;
  int block_size_;       // voxels per block edge
  int padding_;          // voxels within max_distance
  double offset_scale_;  // [m] per quantization step of the site offsets
  /// Site offsets (x, y, z) of all voxels, block after block, x fastest within a block.
  std::vector<int8_t> site_offsets_;
  /// Block coordinate -> index of the block in site_offsets_.
  std::unordered_map<Eigen::Vector3i, uint32_t, KeyHash> block_index_;
};

}  // namespace amcl_3d
