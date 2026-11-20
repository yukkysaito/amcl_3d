#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "amcl_3d_core/map/gaussian_voxel_map.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood.hpp"

namespace amcl_3d
{
/// Distribution-to-distribution likelihood of voxelized GICP (Segal et al., 2009; Koide et al.,
/// 2021). Like NdtLikelihood, but each scan point also carries the covariance C_k of the scan
/// around it, rotated with the particle:
///   log p(z | x) = sum_k -0.5 * min(e_k^T (Sigma_v + R C_k R^T + sigma^2 I)^-1 e_k, max^2)
///   e_k = R p_k + t - mu_v
/// where v is the candidate voxel (GaussianVoxelMap::candidates()) that explains the point best
/// by the map distribution alone. Both surfaces being modeled, an error along a surface shared by
/// the scan and the map is cheap and an error across it is penalized, which makes the score a
/// smooth plane-to-plane distance.
///
/// The scan covariances are estimated from the `neighbor_num` nearest points in the whole filtered
/// scan once per scan (createMeasurementModel()), since the sampled points alone are too sparse.
class GicpLikelihood : public PointCloudLikelihood
{
public:
  struct Params
  {
    double sigma = 0.2;                     // [m] measurement noise added to every pair
    double max_mahalanobis_distance = 5.0;  // per-point clipping, in standard deviations
    size_t neighbor_num = 10;               // scan points per scan covariance
    /// CPU threads of the evaluation (0: all hardware threads). See LikelihoodField::Params.
    size_t thread_num = 4;
  };

  /// Throws std::invalid_argument if a parameter is not positive (neighbor_num < 3).
  static void validate(const Params & params);

  /// Throws std::invalid_argument if the map is null or validate() fails.
  GicpLikelihood(std::shared_ptr<const GaussianVoxelMap> map, const Params & params);

  /// Covariances of `points` from their nearest neighbors in `scan` (both in the same frame).
  std::vector<Eigen::Matrix3d> estimateCovariances(
    const std::vector<Eigen::Vector3d> & scan, const std::vector<Eigen::Vector3d> & points) const;

  /// Log-likelihood with the given scan covariance of each point.
  Eigen::VectorXd logLikelihood(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points,
    const std::vector<Eigen::Matrix3d> & covariances) const;

  /// With covariances estimated from `points` themselves (prefer createMeasurementModel()).
  Eigen::VectorXd logLikelihood(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const override;
  std::unique_ptr<MeasurementModel> createMeasurementModel(
    const std::vector<Eigen::Vector3d> & scan, std::vector<Eigen::Vector3d> sampled) const override;
  std::string description() const override;

  /// Clipped squared Mahalanobis distance of a point in the map frame with its rotated scan
  /// covariance.
  double scorePoint(const Eigen::Vector3d & point, const Eigen::Matrix3d & covariance) const;

private:
  std::shared_ptr<const GaussianVoxelMap> map_;
  Params params_;
  double max_squared_distance_;
  /// (Sigma_v + sigma^2 I)^-1 of each voxel, to choose the corresponding voxel.
  std::vector<Eigen::Matrix3d> information_;
};

}  // namespace amcl_3d
