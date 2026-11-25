#pragma once

#include <cstddef>

#include "amcl_3d_core/motion_model/motion_model.hpp"

namespace amcl_3d
{
/// Motion of a car-like vehicle from its forward speed and yaw rate (wheel odometry, a vehicle
/// speed sensor, a gyro).
///
/// Compared with TwistMotionModel, the motion and its uncertainty follow how cars move:
/// - Non-holonomic: the vehicle moves along its heading. Only the forward speed v (twist.linear.x)
///   and the yaw rate w (twist.angular.z) are used; the other components are ignored. The base
///   frame is expected at the rear axle center, where the lateral velocity of a car is zero.
/// - The noise is anisotropic: forward speed and yaw rate errors (scale and bias, e.g. tire radius
///   and gyro bias), lateral and vertical slip that grow with the speed, and attitude changes
///   (road grade and bank) that random walk with the distance traveled: their std grows with
///   sqrt(distance), so their rate noise with sqrt(|v|). A rate noise proportional to |v| made
///   roll and pitch diffuse by ~14 deg per second at highway speed and the filter overconfident
///   (NEES in the hundreds on KITTI 01). The vertical slip absorbs a
///   misaligned base frame and the suspension: without it, a base frame pitched by 0.8 deg against
///   the direction of travel made the estimate sink by 0.1 m/s and diverge on KITTI.
/// - The noise is computed as if the vehicle moved at least `min_noise_speed`. A stopped car does
///   not move, but the particles keep a small diffusion so that the estimate still refines: without
///   it the first scan collapses the cloud onto its best initial sample (likelihoods of hundreds of
///   points are much sharper than the particle spacing) and a car initialized while parked stays
///   off by that sample's error (1.7 deg in the tests).
///
/// All noise is white noise per sqrt(second) as in TwistMotionModel, so the diffusion does not
/// depend on the prediction rate. Particles are moved in parallel with per-particle random streams.
class VehicleMotionModel : public MotionModel
{
public:
  struct Params
  {
    double speed_noise_scale = 0.1;     // [1/sqrt(s)] of |v|
    double speed_noise_bias = 0.1;      // [m/s * sqrt(s)]
    double yaw_rate_noise_scale = 0.1;  // [1/sqrt(s)] of |w|
    double yaw_rate_noise_bias = 0.02;  // [rad/s * sqrt(s)]
    double lateral_noise_scale = 0.1;   // [1/sqrt(s)] lateral velocity per |v|
    double vertical_noise_scale = 0.1;  // [1/sqrt(s)] vertical velocity per |v|
    /// [rad/sqrt(m)] roll / pitch random walk per sqrt(distance traveled)
    double attitude_noise_scale = 0.01;
    double min_noise_speed = 1.0;        // [m/s] noise floor: speed assumed for the noise
    size_t thread_num = 4;               // CPU threads (0: all hardware threads)
  };

  VehicleMotionModel() = default;
  explicit VehicleMotionModel(const Params & params) : params_(params) {}

  void predict(
    const Twist & twist, double dt, ParticleSet & particles, RandomGenerator & rng) const override;
  std::optional<LinearizedMotion> linearize(const Twist & twist, double dt) const override;

private:
  Params params_;
};

}  // namespace amcl_3d
