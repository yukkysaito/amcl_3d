#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood.hpp"

namespace amcl_3d
{
/// Likelihood field model (Thrun et al., Probabilistic Robotics, 6.4): Gaussian likelihood of the
/// distance from each point to the nearest map point,
///   log p(z | x) = sum_k -min(d_k^2, max_distance^2) / (2 * sigma^2)
/// where d_k is the distance of point k, transformed by the particle pose, to the map.
/// Clipping at `max_distance` makes the model robust to points not in the map (dynamic objects).
/// The distance queries are delegated to a PointCloudMap (distance field, k-d tree, GPU).
class LikelihoodField : public PointCloudLikelihood
{
public:
  struct Params
  {
    double max_distance = 1.0;  // [m]
    double sigma = 0.2;         // [m]
    /// CPU threads of the evaluation (0: all hardware threads; ignored by GPU maps). Small on
    /// purpose: the localizer shares the CPU with other nodes, and every hardware thread was
    /// measured to be slower than a few, because idle OpenMP threads spin on the cores the rest of
    /// the update runs on (see README).
    size_t thread_num = 4;
  };

  /// Throws std::invalid_argument if the map is null, sigma <= 0 or max_distance is beyond the
  /// distance range of the map.
  LikelihoodField(std::shared_ptr<const PointCloudMap> map, const Params & params);

  Eigen::VectorXd logLikelihood(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const override;
  std::string description() const override;

  const Params & params() const { return params_; }

private:
  std::shared_ptr<const PointCloudMap> map_;
  Params params_;
};

}  // namespace amcl_3d
