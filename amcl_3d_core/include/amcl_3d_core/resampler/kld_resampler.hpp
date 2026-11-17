#pragma once

#include "amcl_3d_core/resampler/resampler.hpp"

namespace amcl_3d
{
/// KLD sampling (Fox, 2003): draws particles until their count is large enough that the
/// Kullback-Leibler divergence between the sample-based and the true posterior is below `epsilon`
/// with probability 1 - `delta`. The required count grows with the number of occupied
/// histogram bins, i.e. many particles while uncertain and few once converged.
class KldResampler : public Resampler
{
public:
  struct Params
  {
    size_t min_particle_num = 100;
    size_t max_particle_num = 5000;
    double epsilon = 0.05;
    double delta = 0.01;
    double bin_size_xyz = 0.5;  // [m]
    double bin_size_yaw = 0.2;  // [rad]
  };

  /// Throws std::invalid_argument for inconsistent parameters.
  explicit KldResampler(const Params & params);

  std::vector<size_t> drawIndices(
    const ParticleSet & particles, RandomGenerator & rng) const override;

  /// Number of particles required for `occupied_bin_num` bins.
  double requiredParticleNum(size_t occupied_bin_num) const;

private:
  Params params_;
  double z_;  // upper (1 - delta) quantile of the standard normal distribution
};

}  // namespace amcl_3d
