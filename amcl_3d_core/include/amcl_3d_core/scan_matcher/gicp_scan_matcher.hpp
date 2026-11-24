#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "amcl_3d_core/scan_matcher/scan_matcher.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Generalized ICP (Segal et al., 2009) by Gauss-Newton on SE(3), in double precision.
/// Every map point and every scan point carries the covariance of its neighborhood, regularized to
/// a plane, so the cost is a plane-to-plane distance that pulls hard across surfaces and lets the
/// scan slide along them:
///   E(T) = sum_k rho(e_k^T (C_m + R C_k R^T)^-1 e_k),  e_k = R p_k + t - m_k
/// with a Cauchy kernel rho.
/// m_k is the map point nearest to the transformed scan point (re-associated every iteration);
/// pairs farther than max_correspondence_distance are dropped.
///
/// The map covariances are computed once (memory: a k-d tree and 6 floats per map point). A
/// per-point map keeps the cm-level geometry that voxel distributions blur: on KITTI, aligning
/// from the ground truth stays within a few cm, a 1 m voxelized GICP drifts 5-10 cm.
class GicpScanMatcher : public ScanMatcher
{
public:
  struct Params
  {
    double max_correspondence_distance = 1.0;  // [m]
    size_t neighbor_num = 10;                  // points per covariance (map and scan)
    int max_iterations = 30;
    /// Converged when an update is smaller than both. Nearest-neighbor re-association makes the
    /// last updates oscillate around 1e-4, so tighter thresholds mostly reject good results.
    double translation_epsilon = 1e-3;  // [m]
    double rotation_epsilon = 1e-3;     // [rad]
    /// Width of the Cauchy kernel on the Mahalanobis distance of a pair: pairs much farther
    /// (moving objects, map changes) are down-weighted instead of pulling the scan.
    double robust_kernel_width = 3.0;
    /// Results whose fraction of scan points with a correspondence is lower are not converged.
    double min_inlier_ratio = 0.3;
    /// Scan points registered at most (an even subset; 0: all). The covariances still come from
    /// the whole scan.
    size_t max_point_num = 1000;
    size_t thread_num = 4;  // CPU threads (0: all hardware threads)
  };

  /// Throws std::invalid_argument for an empty map or a parameter out of range.
  GicpScanMatcher(const PointCloud::ConstPtr & map, const Params & params);

  /// fitness: mean squared Mahalanobis distance of the inliers.
  ScanMatchingResult align(
    const std::vector<Eigen::Vector3d> & scan, const Pose & initial_guess) override;

  std::string description() const;

private:
  Params params_;
  PointCloud::ConstPtr map_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  /// Upper triangle (xx, xy, xz, yy, yz, zz) of each map point's plane covariance.
  std::vector<std::array<float, 6>> map_covariances_;
};

/// Covariance of each point from its `neighbor_num` nearest neighbors in `points`, regularized to
/// a plane (see regularizeToPlane()).
std::vector<Eigen::Matrix3d> planeCovariances(
  const std::vector<Eigen::Vector3d> & points, size_t neighbor_num, size_t thread_num = 1);

/// planeCovariances() of points[queries[i]] only, from their neighbors among all `points`.
std::vector<Eigen::Matrix3d> planeCovariances(
  const std::vector<Eigen::Vector3d> & points, const std::vector<size_t> & queries,
  size_t neighbor_num, size_t thread_num = 1);

/// Eigenvalues replaced by (1e-3, 1, 1) along the principal axes, the smallest one being the
/// surface normal (Segal et al., 2009): the covariance of a plane, whatever the point density.
Eigen::Matrix3d regularizeToPlane(const Eigen::Matrix3d & covariance);

}  // namespace amcl_3d
