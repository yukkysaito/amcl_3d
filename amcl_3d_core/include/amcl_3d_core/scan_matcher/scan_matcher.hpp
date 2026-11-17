#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
struct ScanMatchingResult
{
  bool converged = false;
  Pose pose;
  /// Implementation specific matching error (lower is better).
  double fitness = std::numeric_limits<double>::infinity();
};

/// Registers a scan (base frame) to the map, starting from an initial guess.
/// Used to refine the particle filter estimate beyond the resolution of the particle cloud.
class ScanMatcher
{
public:
  virtual ~ScanMatcher() = default;
  virtual ScanMatchingResult align(
    const std::vector<Eigen::Vector3d> & scan, const Pose & initial_guess) = 0;
};

}  // namespace amcl_3d
