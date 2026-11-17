#pragma once
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"
#include "amcl_3d/xorshift128.hpp"
#include "amcl_3d/mcl.hpp"

namespace amcl_3d
{
// 2D constant twist (arc) motion model driven by forward velocity vel.x and yaw rate omega.z.
// Velocity noise is modeled as white noise, so the displacement std grows with sqrt(dt) and
// the resulting diffusion does not depend on how often predict() is called.
class FooPredictionModel : public PredictionModelInterface
{
public:
  struct NoiseParam
  {
    double vel_scale = 0.1;    // [1/sqrt(s)] std of velocity noise proportional to |vel|
    double vel_bias = 0.1;     // [m/s*sqrt(s)] std of velocity noise independent of vel
    double omega_scale = 0.1;  // [1/sqrt(s)] std of yaw rate noise proportional to |omega|
    double omega_bias = 0.02;  // [rad/s*sqrt(s)] std of yaw rate noise independent of omega
  };

  FooPredictionModel();
  FooPredictionModel(const Eigen::Vector3d &vel, const Eigen::Vector3d &omega);
  FooPredictionModel(const Eigen::Vector3d &vel, const Eigen::Vector3d &omega, const NoiseParam &noise_param);
  bool predict(State &state, const double dt_sec) override;
  bool measumentLinearVelocity(const Eigen::Vector3d &vel);
  bool measumentAngularVelocity(const Eigen::Vector3d &omega);
  Eigen::Vector3d getLinearVelocity();
  Eigen::Vector3d getAngularVelocity();
  void setNoiseParam(const NoiseParam &noise_param) { noise_param_ = noise_param; }
  NoiseParam getNoiseParam() const { return noise_param_; }

private:
  Eigen::Vector3d vel_;
  Eigen::Vector3d omega_;
  NoiseParam noise_param_;
  static XorShift128 rand_;
};

} // namespace amcl_3d
