#include "amcl_3d_core/particle_clustering.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace amcl_3d
{
namespace
{
using BinKey = std::array<int64_t, 4>;  // x, y, z, yaw

struct BinKeyHash
{
  size_t operator()(const BinKey & key) const
  {
    size_t hash = 0;
    for (const int64_t k : key) {
      hash = hash * 1000003u ^ static_cast<size_t>(k);
    }
    return hash;
  }
};

class DisjointSets
{
public:
  explicit DisjointSets(size_t size) : parent_(size)
  {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  size_t find(size_t i)
  {
    while (parent_[i] != i) {
      parent_[i] = parent_[parent_[i]];  // path halving
      i = parent_[i];
    }
    return i;
  }

  void unite(size_t a, size_t b)
  {
    a = find(a);
    b = find(b);
    if (a != b) {
      parent_[std::max(a, b)] = std::min(a, b);
    }
  }

private:
  std::vector<size_t> parent_;
};
}  // namespace

ParticleClustering clusterParticles(
  const ParticleSet & particles, const ParticleClustering::Params & params)
{
  if (!(params.bin_size_xyz > 0.0) || !(params.bin_size_yaw > 0.0)) {
    throw std::invalid_argument("clusterParticles: bin sizes must be positive");
  }
  const int64_t yaw_bin_num =
    std::max<int64_t>(1, static_cast<int64_t>(std::ceil(2.0 * M_PI / params.bin_size_yaw)));
  const double yaw_bin_size = 2.0 * M_PI / static_cast<double>(yaw_bin_num);

  std::unordered_map<BinKey, size_t, BinKeyHash> bin_index;
  bin_index.reserve(particles.size());
  std::vector<BinKey> bins;
  std::vector<size_t> bin_of_particle(particles.size());
  for (size_t i = 0; i < particles.size(); ++i) {
    const Pose pose = particles.pose(i);
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));  // in [-pi, pi]
    BinKey key;
    for (int axis = 0; axis < 3; ++axis) {
      // NaN and huge values all land in one bin instead of overflowing
      const double scaled = pose.position[axis] / params.bin_size_xyz;
      key[static_cast<size_t>(axis)] =
        std::abs(scaled) < 1e15 ? static_cast<int64_t>(std::floor(scaled)) : INT64_MIN;
    }
    const auto yaw_bin = static_cast<int64_t>(std::floor((yaw + M_PI) / yaw_bin_size));
    key[3] = std::isfinite(yaw) ? (yaw_bin % yaw_bin_num + yaw_bin_num) % yaw_bin_num : 0;
    const auto [it, inserted] = bin_index.emplace(key, bins.size());
    if (inserted) {
      bins.push_back(key);
    }
    bin_of_particle[i] = it->second;
  }

  // Connect each bin to its 80 neighbors. The relation is symmetric, so looking at the 40 offsets
  // that are lexicographically positive is enough.
  std::vector<BinKey> offsets;
  for (int64_t dx = -1; dx <= 1; ++dx) {
    for (int64_t dy = -1; dy <= 1; ++dy) {
      for (int64_t dz = -1; dz <= 1; ++dz) {
        for (int64_t dyaw = -1; dyaw <= 1; ++dyaw) {
          const BinKey offset{dx, dy, dz, dyaw};
          if (offset > BinKey{0, 0, 0, 0}) {
            offsets.push_back(offset);
          }
        }
      }
    }
  }
  DisjointSets sets(bins.size());
  for (size_t b = 0; b < bins.size(); ++b) {
    const BinKey & key = bins[b];
    for (const BinKey & offset : offsets) {
      const BinKey neighbor{
        key[0] + offset[0], key[1] + offset[1], key[2] + offset[2],
        (key[3] + offset[3] + yaw_bin_num) % yaw_bin_num};
      const auto it = bin_index.find(neighbor);
      if (it != bin_index.end()) {
        sets.unite(b, it->second);
      }
    }
  }

  ParticleClustering clustering;
  clustering.labels.resize(particles.size());
  std::unordered_map<size_t, size_t> label_of_root;
  for (size_t i = 0; i < particles.size(); ++i) {
    const auto [it, inserted] =
      label_of_root.emplace(sets.find(bin_of_particle[i]), label_of_root.size());
    clustering.labels[i] = it->second;
  }
  clustering.cluster_num = label_of_root.size();
  return clustering;
}

}  // namespace amcl_3d
