#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"
#include "amcl_3d_core/motion_model/motion_model_factory.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/resampler/kld_resampler.hpp"
#include "amcl_3d_core/scan_filter.hpp"
#include "amcl_3d_core/scan_matcher/scan_matcher.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
enum class ResamplerType
{
  kSystematic,
  kKld
};

struct LocalizerParams
{
  size_t initial_particle_num = 500;
  /// Applied to every scan before sampling.
  ScanFilter::Params scan_filter;
  /// Filtered scan points randomly selected per update. 0 uses all points.
  size_t scan_sample_num = 200;
  MotionModelParams motion_model;
  PointCloudMeasurementModel::Params point_cloud_measurement;

  struct Resampling
  {
    ResamplerType type = ResamplerType::kSystematic;
    /// Resample when ESS / N drops below this ratio. 1.0 resamples at every update.
    double ess_ratio_threshold = 0.5;
    KldResampler::Params kld;
  } resampling;

  /// Augmented MCL (Thrun et al., Probabilistic Robotics, Table 8.3): when the short-term average
  /// likelihood w_fast drops below the long-term one w_slow, a ratio max(0, 1 - w_fast / w_slow)
  /// of the particles is perturbed at resampling to recover from localization failure.
  struct AugmentedMcl
  {
    bool enabled = false;
    double alpha_slow = 0.001;
    double alpha_fast = 0.1;
    /// Perturbation std of injected particles: x, y, z [m], roll, pitch, yaw [rad].
    Vector6d injection_stddev = (Vector6d() << 1.0, 1.0, 0.1, 0.05, 0.05, 0.5).finished();
  } augmented_mcl;

  /// Expansion resetting (Ueda et al., emcl): when even the best particle explains the scan poorly
  /// (per-point likelihood below the threshold), all particles are spread by `stddev` instead of
  /// being resampled. Takes precedence over augmented MCL.
  struct ExpansionReset
  {
    bool enabled = false;
    /// Per-point likelihood exp(-d^2 / (2 sigma^2)) averaged in the log domain.
    double likelihood_threshold = 0.05;
    /// x, y, z [m], roll, pitch, yaw [rad]
    Vector6d stddev = (Vector6d() << 0.3, 0.3, 0.05, 0.01, 0.01, 0.1).finished();
  } expansion_reset;

  /// Refinement of the estimate by scan matching; active when a ScanMatcher is set.
  struct ScanMatching
  {
    /// Matched poses farther than this from the particle filter estimate are rejected [m].
    double max_correction = 1.0;
    /// Also correct the particles with the matched pose. The scan is then used twice, which makes
    /// the filter overconfident, but keeps the particles close to the refined pose.
    bool feedback = false;
    /// Std of the matched pose used for the feedback: x, y, z [m], roll, pitch, yaw [rad].
    Vector6d feedback_stddev = (Vector6d() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.02).finished();
  } scan_matching;

  uint64_t random_seed = 0;
};

enum class LocalizerStatus
{
  kOk,
  kNotInitialized,
  kNoMap,
  kStaleStamp,          // older than the current filter time
  kInvalidMeasurement,  // e.g. empty scan, non positive definite covariance
};
const char * toString(LocalizerStatus status);

/// Monte Carlo localization against a 3D point cloud map.
///
/// Typical usage:
///   setMap() -> initialize() -> { addTwist() at odometry rate, updateWithPointCloud() per scan }
/// Twists are buffered and integrated piecewise up to each measurement stamp, so odometry may
/// arrive ahead of the (usually delayed) scans.
class Localizer
{
public:
  /// Throws std::invalid_argument for inconsistent parameters.
  explicit Localizer(const LocalizerParams & params);

  // Extension points: replace the models built from the parameters.
  void setMotionModel(std::unique_ptr<MotionModel> model);
  void setResampler(std::unique_ptr<Resampler> resampler);
  /// Enables refinement by scan matching (nullptr disables it).
  void setScanMatcher(std::unique_ptr<ScanMatcher> matcher) { scan_matcher_ = std::move(matcher); }

  /// Throws std::invalid_argument if the map is null or its distance range is shorter than
  /// point_cloud_measurement.max_distance.
  void setMap(std::shared_ptr<const PointCloudMap> map);
  bool hasMap() const { return map_ != nullptr; }

  void initialize(const PoseWithCovariance & initial_pose, const Time & stamp);
  /// Initializes from an arbitrary distribution, e.g. GroundPoseSampler for global localization.
  /// Prefer KLD resampling then, so that the particle count shrinks after convergence.
  void initialize(const PoseSampler & sampler, size_t particle_num, const Time & stamp);
  bool isInitialized() const { return initialized_; }

  /// Registers a body twist valid from `stamp` until the next one. Twists must be added in time
  /// order; returns false (and ignores the twist) otherwise.
  bool addTwist(const Twist & twist, const Time & stamp);

  /// Moves the particles forward to `stamp` with the buffered twists.
  LocalizerStatus predict(const Time & stamp);

  /// predict(stamp), then corrects with a scan given in the base frame, then resamples if needed,
  /// then refines the estimate by scan matching if a ScanMatcher is set.
  LocalizerStatus updateWithPointCloud(const PointCloud & scan, const Time & stamp);

  /// predict(stamp), then corrects with an absolute pose observation (e.g. NDT, GNSS).
  LocalizerStatus updateWithPose(const PoseWithCovariance & observation, const Time & stamp);

  PoseWithCovariance estimate() const { return filter_.estimate(); }
  /// Pose refined by scan matching at the last scan update, if it succeeded.
  const std::optional<Pose> & refinedPose() const { return refined_pose_; }
  const ParticleSet & particles() const { return filter_.particles(); }
  double effectiveSampleSize() const { return filter_.effectiveSampleSize(); }
  /// Likelihood of the last measurement update, for diagnostics.
  const LikelihoodSummary & lastLikelihood() const { return last_likelihood_; }
  /// Number of expansion resets since initialization, for diagnostics.
  size_t expansionResetCount() const { return expansion_reset_count_; }
  /// Time the particles currently represent.
  const Time & stamp() const { return stamp_; }

private:
  void resetState(const Time & stamp);
  void correct(const MeasurementModel & model);
  std::vector<Eigen::Vector3d> samplePoints(const std::vector<Eigen::Vector3d> & points);
  void refineByScanMatching(const std::vector<Eigen::Vector3d> & points);
  void pruneTwists();

  LocalizerParams params_;
  ParticleFilter filter_;
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<Resampler> resampler_;
  std::unique_ptr<ScanMatcher> scan_matcher_;
  GaussianPoseSampler injection_sampler_;
  GaussianPoseSampler expansion_sampler_;
  ScanFilter scan_filter_;
  std::shared_ptr<const PointCloudMap> map_;

  bool initialized_ = false;
  Time stamp_;
  /// Time ordered. After pruning, at most the front entry is at or before stamp_ (the active one).
  std::deque<std::pair<Time, Twist>> twists_;
  double w_slow_ = 0.0;
  double w_fast_ = 0.0;
  LikelihoodSummary last_likelihood_;
  size_t expansion_reset_count_ = 0;
  std::optional<Pose> refined_pose_;
};

}  // namespace amcl_3d
