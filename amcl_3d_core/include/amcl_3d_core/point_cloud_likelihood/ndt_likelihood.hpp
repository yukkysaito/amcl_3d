#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "amcl_3d_core/map/gaussian_voxel_map.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood.hpp"

namespace amcl_3d
{
/// NDT likelihood (Biber & Strasser, 2003; Magnusson, 2009): each point, transformed by the
/// particle pose, is scored by the normal distribution of the map points in its voxel or a
/// face-neighboring one, whichever explains it best:
///   log p(z | x) = sum_k -0.5 * min(min_v m_kv, max_mahalanobis_distance^2)
///   m_kv = (q_k - mu_v)^T (Sigma_v + sigma^2 I)^-1 (q_k - mu_v),  q_k = R p_k + t
/// The distributions model the local surface shape: a point is penalized by its distance to the
/// surface (normal direction, sharp) much more than along it (spread of the voxel). `sigma` adds
/// the measurement noise and keeps the flat distributions of planes invertible. Clipping makes the
/// model robust to points not in the map, as in LikelihoodField.
///
/// Compared with the likelihood field, the map is far smaller (one distribution per 1 m voxel
/// instead of a dense distance field) and the score is smooth along surfaces.
class NdtLikelihood : public PointCloudLikelihood
{
public:
  struct Params
  {
    double sigma = 0.2;                     // [m] measurement noise added to every distribution
    double max_mahalanobis_distance = 5.0;  // per-point clipping, in standard deviations
    /// Also score with the 6 face-neighboring voxels: widens the basin beyond a voxel and removes
    /// the jumps at voxel boundaries, for 7 times the lookups.
    bool search_neighbors = true;
    /// CPU threads of the evaluation (0: all hardware threads). See LikelihoodField::Params.
    size_t thread_num = 4;
  };

  /// Throws std::invalid_argument if a parameter is not positive.
  static void validate(const Params & params);

  /// Throws std::invalid_argument if the map is null or validate() fails.
  NdtLikelihood(std::shared_ptr<const GaussianVoxelMap> map, const Params & params);

  Eigen::VectorXd logLikelihood(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const override;
  std::string description() const override;

  /// Clipped squared Mahalanobis distance of a point in the map frame to its best distribution.
  double scorePoint(const Eigen::Vector3d & point) const;

private:
  struct Candidates
  {
    uint32_t begin;
    uint32_t count;
  };

  std::shared_ptr<const GaussianVoxelMap> map_;
  Params params_;
  double max_squared_distance_;
  /// (Sigma_v + sigma^2 I)^-1 of each voxel of the map, in the order of map_->voxels().
  std::vector<Eigen::Matrix3d> information_;
  /// Voxels scoring a point, per grid cell: the cell itself and its face neighbors that hold a
  /// distribution. Precomputed so that a point costs one hash lookup.
  std::unordered_map<Eigen::Vector3i, Candidates, GaussianVoxelMap::CoordinateHash> candidates_;
  std::vector<uint32_t> candidate_voxels_;
};

}  // namespace amcl_3d
