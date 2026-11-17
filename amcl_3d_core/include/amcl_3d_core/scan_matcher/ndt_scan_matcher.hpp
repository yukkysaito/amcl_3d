#pragma once

#include <pcl/registration/ndt.h>

#include "amcl_3d_core/scan_matcher/scan_matcher.hpp"

namespace amcl_3d
{
/// Normal Distributions Transform (Magnusson, 2009) by PCL.
/// PCL works in single precision: for maps in large coordinates (e.g. UTM), shift the map close
/// to the origin first.
class NdtScanMatcher : public ScanMatcher
{
public:
  struct Params
  {
    double resolution = 1.0;  // [m] NDT voxel size
    double step_size = 0.1;   // [m] maximum line search step
    double transformation_epsilon = 0.01;
    int max_iterations = 30;
    /// Results with a larger PCL fitness score (mean squared point distance) are not converged.
    double max_fitness = 1.0;  // [m^2]
  };

  /// Builds the NDT voxels of the map once. Throws std::invalid_argument for an empty map.
  NdtScanMatcher(const PointCloud::ConstPtr & map, const Params & params);

  ScanMatchingResult align(
    const std::vector<Eigen::Vector3d> & scan, const Pose & initial_guess) override;

private:
  Params params_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
};

}  // namespace amcl_3d
