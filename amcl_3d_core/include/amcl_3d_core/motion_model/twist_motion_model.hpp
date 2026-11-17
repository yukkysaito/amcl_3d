#pragma once

#include "amcl_3d_core/motion_model/motion_model.hpp"

namespace amcl_3d
{
/// Constant body twist model: each particle moves by the SE(3) exponential of (v, w) * dt.
/// For planar motion this is the classic arc (unicycle) model, and it naturally extends to 3D
/// motion such as driving on slopes when the angular velocity comes from an IMU.
///
/// Velocity noise is white noise with standard deviation `scale * |v| + bias` per sqrt(second),
/// applied to each axis. The resulting displacement noise grows with sqrt(dt), so the diffusion
/// does not depend on how often predict() is called.
class TwistMotionModel : public MotionModel
{
public:
  struct Params
  {
    double linear_noise_scale = 0.1;   // [1/sqrt(s)]
    double linear_noise_bias = 0.1;    // [m/s * sqrt(s)]
    double angular_noise_scale = 0.1;  // [1/sqrt(s)]
    double angular_noise_bias = 0.02;  // [rad/s * sqrt(s)]
  };

  TwistMotionModel() = default;
  explicit TwistMotionModel(const Params & params) : params_(params) {}

  void predict(
    const Twist & twist, double dt, ParticleSet & particles, RandomGenerator & rng) const override;

  /// Moves a single pose without noise.
  static Pose integrate(const Pose & pose, const Twist & twist, double dt);
  /// Inverse of integrate(): the constant body twist that moves `from` to `to` in `dt` seconds.
  static Twist twistBetween(const Pose & from, const Pose & to, double dt);

private:
  Params params_;
};

}  // namespace amcl_3d
