#pragma once

#include <cstddef>
#include <vector>

#include "amcl_3d_core/motion_model/motion_model.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Density of each particle's pose under the prediction since the last measurement, which MCMC
/// moves need: the predicted distribution is a mixture over the previous particles and cannot be
/// evaluated at an arbitrary pose, but on the extended space (ancestor, pose) of resample-move
/// (Gilks & Berzuini, 2001) the target of a particle is p(x | its ancestor, odometry) L(x)^b.
///
/// Each particle keeps its anchor, the noise-free prediction of its ancestor, and the pose is
/// retract(anchor, eps) with eps ~ N(0, covariance) in the anchor's frame. The covariance is shared
/// (the noise depends on the odometry, not the pose) and propagated through every prediction step
/// to first order: covariance <- A covariance A^T + noise of the step, A the adjoint of the step.
/// For a single step per measurement interval this is the linearized density of the motion model.
///
/// Kinds: kGaussian as above; kFlat, a constant density (after a global initialization, the
/// prior is about uniform over the map); kNone, unknown (a motion model without linearize()).
class TransitionPrior
{
public:
  enum class Kind
  {
    kNone,
    kFlat,
    kGaussian,
  };

  /// Starts a measurement interval: anchors at the particles, no noise yet.
  void startAt(const ParticleSet & particles);
  /// Every particle drawn from N(mean, covariance) (world frame, as GaussianPoseSampler).
  void startGaussian(size_t particle_num, const PoseWithCovariance & distribution);
  void startFlat(size_t particle_num);
  void invalidate();

  /// Follows one prediction step; invalidates the prior if the model cannot linearize it.
  void predict(const MotionModel & model, const Twist & twist, double dt);
  /// Follows resampling: particle j is a copy of particle indices[j].
  void select(const std::vector<size_t> & indices);

  Kind kind() const { return kind_; }
  bool available() const { return kind_ != Kind::kNone; }
  const Matrix6d & covariance() const { return covariance_; }
  const Pose & anchor(size_t i) const { return anchors_.at(i); }

  /// log p(pose | particle i's ancestor) up to a constant; 0 for kFlat.
  double logDensity(size_t i, const Pose & pose) const;
  /// logDensity() with its gradient and negative Hessian with respect to the local perturbation
  /// of `pose` (retract()), to first order in the distance to the anchor.
  double logDensity(size_t i, const Pose & pose, Vector6d & gradient, Matrix6d & hessian) const;

private:
  void updateInformation();

  Kind kind_ = Kind::kNone;
  std::vector<Pose> anchors_;
  Matrix6d covariance_ = Matrix6d::Zero();
  Matrix6d information_ = Matrix6d::Zero();
};

}  // namespace amcl_3d
