#pragma once

#include <vector>

#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Samples poses over the whole map for global localization of a ground vehicle:
///   x, y : uniform over the grid cells that contain map points
///   z    : lowest map point of the cell (the ground) + base_height
///   yaw  : uniform, roll = pitch = 0
/// Multi-level structures (bridges, parking garages) are not handled.
class GroundPoseSampler
{
public:
  struct Params
  {
    double cell_size = 1.0;    // [m]
    double base_height = 0.0;  // [m] height of the base frame above the ground
  };

  /// Throws std::invalid_argument for an empty map or a non positive cell size.
  GroundPoseSampler(const PointCloud & map, const Params & params);

  Pose sample(RandomGenerator & rng) const;
  size_t cellNum() const { return cells_.size(); }

private:
  struct Cell
  {
    Eigen::Vector2d min_corner;
    double ground_z;
  };
  Params params_;
  std::vector<Cell> cells_;
};

}  // namespace amcl_3d
