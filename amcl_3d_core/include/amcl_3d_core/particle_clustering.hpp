#pragma once

#include <cstddef>
#include <vector>

#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
/// Groups particles into clusters, so that a multimodal particle cloud (e.g. during global
/// localization or in a symmetric environment) is summarized by its most likely mode instead of
/// a mean that lies between the modes. Same idea as the kd-tree clusters of ROS amcl.
///
/// Particles are binned on a grid over x, y, z and yaw; a cluster is a connected component of
/// occupied bins, where bins that touch, including diagonally, are connected and yaw wraps around.
/// Roll and pitch are ignored: they are small for ground vehicles.
struct ParticleClustering
{
  struct Params
  {
    double bin_size_xyz = 0.5;  // [m]
    double bin_size_yaw = 0.2;  // [rad], rounded down to divide 2 pi
  };

  /// Cluster index of every particle, numbered from 0 in order of first appearance.
  std::vector<size_t> labels;
  size_t cluster_num = 0;
};

ParticleClustering clusterParticles(
  const ParticleSet & particles, const ParticleClustering::Params & params);

}  // namespace amcl_3d
