#include "amcl_3d_core/pose_sampler/ground_pose_sampler.hpp"

#include <cmath>
#include <cstdint>
#include <map>
#include <random>
#include <stdexcept>
#include <utility>

namespace amcl_3d
{
GroundPoseSampler::GroundPoseSampler(const PointCloud & map, const Params & params)
: params_(params)
{
  if (params_.cell_size <= 0.0) {
    throw std::invalid_argument("GroundPoseSampler: cell_size must be positive");
  }
  std::map<std::pair<int64_t, int64_t>, double> ground_z;  // ordered: deterministic sampling
  for (const auto & point : map) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const auto key = std::make_pair(
      static_cast<int64_t>(std::floor(point.x / params_.cell_size)),
      static_cast<int64_t>(std::floor(point.y / params_.cell_size)));
    const auto [it, inserted] = ground_z.emplace(key, point.z);
    if (!inserted && point.z < it->second) {
      it->second = point.z;
    }
  }
  if (ground_z.empty()) {
    throw std::invalid_argument("GroundPoseSampler: the map has no finite point");
  }
  cells_.reserve(ground_z.size());
  for (const auto & [key, z] : ground_z) {
    cells_.push_back(Cell{
      Eigen::Vector2d(static_cast<double>(key.first), static_cast<double>(key.second)) *
        params_.cell_size,
      z});
  }
}

Pose GroundPoseSampler::sample(RandomGenerator & rng) const
{
  std::uniform_int_distribution<size_t> cell_distribution(0, cells_.size() - 1);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> yaw_distribution(-M_PI, M_PI);

  const Cell & cell = cells_[cell_distribution(rng)];
  Pose pose;
  pose.position.x() = cell.min_corner.x() + unit(rng) * params_.cell_size;
  pose.position.y() = cell.min_corner.y() + unit(rng) * params_.cell_size;
  pose.position.z() = cell.ground_z + params_.base_height;
  pose.orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw_distribution(rng), Eigen::Vector3d::UnitZ()));
  return pose;
}

}  // namespace amcl_3d
