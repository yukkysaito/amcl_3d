#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Preprocesses a scan in the base frame before the likelihood evaluation:
///   1. drops non-finite points
///   2. keeps points with min_range <= |p| <= max_range and min_z <= z <= max_z
///      (e.g. removes the vehicle body and far, noisy returns)
///   3. voxel grid downsampling to the centroid of each voxel, which makes the density uniform so
///      that nearby ground points do not dominate the likelihood
/// The default parameters pass every finite point through.
class ScanFilter
{
public:
  struct Params
  {
    double min_range = 0.0;                                 // [m]
    double max_range = std::numeric_limits<double>::max();  // [m]
    double min_z = std::numeric_limits<double>::lowest();   // [m]
    double max_z = std::numeric_limits<double>::max();      // [m]
    double voxel_size = 0.0;                                // [m] 0 disables downsampling
  };

  ScanFilter() = default;
  explicit ScanFilter(const Params & params) : params_(params) {}

  std::vector<Eigen::Vector3d> apply(const PointCloud & scan) const;

private:
  Params params_;
};

}  // namespace amcl_3d
