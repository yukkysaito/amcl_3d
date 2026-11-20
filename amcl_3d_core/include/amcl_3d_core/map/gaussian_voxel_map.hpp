#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Normal distribution of the map points in each voxel of a regular grid, the map representation
/// of NDT (Biber & Strasser, 2003; Magnusson, 2009) and voxelized GICP. Sparse: only voxels with
/// enough points are stored. Immutable once built.
class GaussianVoxelMap
{
public:
  struct Params
  {
    double resolution = 1.0;   // [m] voxel size
    size_t min_point_num = 6;  // voxels with fewer points are dropped
    /// Match points also with the 6 face-neighboring voxels: widens the basin beyond a voxel and
    /// removes the jumps at voxel boundaries, for up to 7 times the work per point.
    bool search_neighbors = true;
  };

  /// Indices in voxels() of the distributions a point is matched with.
  class CandidateRange
  {
  public:
    CandidateRange(const uint32_t * begin, const uint32_t * end) : begin_(begin), end_(end) {}
    const uint32_t * begin() const { return begin_; }
    const uint32_t * end() const { return end_; }
    bool empty() const { return begin_ == end_; }

  private:
    const uint32_t * begin_;
    const uint32_t * end_;
  };

  struct Voxel
  {
    Eigen::Vector3i coordinate;  // grid coordinate, see voxelCoordinate()
    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;  // maximum likelihood estimate (divided by the point count)
    size_t point_num;
  };

  /// Throws std::invalid_argument for non positive parameters.
  static void validate(const Params & params);

  /// Throws std::invalid_argument if validate() fails, for a cloud beyond the grid range
  /// (2^30 voxels from the origin) or a cloud with no voxel of min_point_num points.
  GaussianVoxelMap(const PointCloud & cloud, const Params & params);

  /// Grid coordinate of the voxel containing `point`. Points beyond the grid range map to a
  /// coordinate with no voxel.
  Eigen::Vector3i voxelCoordinate(const Eigen::Vector3d & point) const;
  /// Index in voxels() of the voxel at `coordinate`, or -1 if it holds no distribution.
  int64_t find(const Eigen::Vector3i & coordinate) const;

  /// The distributions `point` (map frame) is matched with: those of its voxel and, with
  /// search_neighbors, of its face neighbors. One hash lookup (precomputed per cell).
  CandidateRange candidates(const Eigen::Vector3d & point) const;

  const std::vector<Voxel> & voxels() const { return voxels_; }

  /// Hash of grid coordinates, for maps keyed by voxel.
  struct CoordinateHash
  {
    size_t operator()(const Eigen::Vector3i & key) const;
  };
  const Params & params() const { return params_; }
  std::string description() const;

private:
  struct Span
  {
    uint32_t begin;
    uint32_t count;
  };

  void buildCandidates();

  Params params_;
  std::vector<Voxel> voxels_;
  std::unordered_map<Eigen::Vector3i, uint32_t, CoordinateHash> index_;
  /// Per grid cell, the voxels matched with its points, as a span of candidate_voxels_.
  std::unordered_map<Eigen::Vector3i, Span, CoordinateHash> candidates_;
  std::vector<uint32_t> candidate_voxels_;
};

}  // namespace amcl_3d
