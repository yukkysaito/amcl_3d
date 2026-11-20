#pragma once

#include <functional>

#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Draws an absolute pose, e.g. the initial distribution of the particles.
/// GaussianPoseSampler (around a mean) and GroundPoseSampler (over the whole map) provide them.
using PoseSampler = std::function<Pose(RandomGenerator &)>;

}  // namespace amcl_3d
