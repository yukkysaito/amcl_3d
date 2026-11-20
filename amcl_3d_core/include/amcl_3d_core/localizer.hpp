#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/motion_model/motion_model_factory.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood.hpp"
#include "amcl_3d_core/resampler/kld_resampler.hpp"
#include "amcl_3d_core/scan_filter.hpp"
#include "amcl_3d_core/scan_matcher/scan_matcher.hpp"
#include "amcl_3d_core/sensor.hpp"
#include "amcl_3d_core/twist_history.hpp"
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

  /// Measurements and twists up to this much older than the newest measurement are still fused
  /// at their own time: the filter rewinds to the state before them and replays the newer
  /// measurements (see Localizer). Bounds the delay of the slowest sensor and the memory (one
  /// particle set per measurement within it). 0: measurements older than the filter are rejected.
  double max_measurement_delay = 1.0;  // [s]

  /// Sensor of updateWithPointCloud(): drives recovery. A scan counts as at most 10 independent
  /// points: on KITTI this cut the error by 25-40 % for every likelihood, motion model and
  /// particle count tried (5-10 was best; less converged more slowly), see README.
  SensorParams point_cloud_sensor{"point_cloud", 10, true, 0.05};
  /// Sensor of updateWithPose().
  SensorParams pose_sensor{"pose", 0, false, 0.05};

  struct Resampling
  {
    ResamplerType type = ResamplerType::kSystematic;
    /// Resample when ESS / N drops below this ratio. 1.0 resamples at every update.
    double ess_ratio_threshold = 0.5;
    KldResampler::Params kld;
  } resampling;

  /// Augmented MCL (Thrun et al., Probabilistic Robotics, Table 8.3): when the short-term average
  /// likelihood w_fast drops below the long-term one w_slow, a ratio max(0, 1 - w_fast / w_slow)
  /// of the particles is perturbed at resampling to recover from localization failure. The
  /// averages are kept per sensor that drives recovery; the largest ratio applies.
  struct AugmentedMcl
  {
    bool enabled = false;
    double alpha_slow = 0.001;
    double alpha_fast = 0.1;
    /// Perturbation std of injected particles: x, y, z [m], roll, pitch, yaw [rad].
    Vector6d injection_stddev = (Vector6d() << 1.0, 1.0, 0.1, 0.05, 0.05, 0.5).finished();
  } augmented_mcl;

  /// Expansion resetting (Ueda et al., emcl): when even the best particle explains a measurement
  /// poorly (per-observation likelihood below SensorParams::expansion_reset_threshold), all
  /// particles are spread by `stddev` instead of being resampled. Takes precedence over augmented
  /// MCL. Only sensors with SensorParams::drives_recovery trigger it.
  struct ExpansionReset
  {
    bool enabled = false;
    /// x, y, z [m], roll, pitch, yaw [rad]
    Vector6d stddev = (Vector6d() << 0.3, 0.3, 0.05, 0.01, 0.01, 0.1).finished();
  } expansion_reset;

  /// Refinement of the estimate by scan matching; active when a ScanMatcher is set.
  struct ScanMatching
  {
    /// Matched poses farther than this from the particle filter estimate are rejected [m].
    double max_correction = 1.0;
    /// Also correct the particles with the matched pose. The scan is then used twice, which makes
    /// the filter overconfident, but keeps the particles close to the refined pose. The feedback
    /// is a sensor of its own that does not drive recovery.
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
  kNoMap,               // no PointCloudLikelihood for a point cloud update
  kStaleStamp,          // older than the history kept for delayed measurements
  kInvalidMeasurement,  // e.g. empty scan, non positive definite covariance
};
const char * toString(LocalizerStatus status);

/// Monte Carlo localization in a 3D map, fusing asynchronous and delayed sensors.
///
/// Typical usage:
///   setPointCloudLikelihood() -> initialize() ->
///     { addTwist() at odometry rate, updateWithPointCloud() per scan, update() per other sensor }
///
/// Time: the filter keeps a history of the measurements of the last max_measurement_delay seconds,
/// each with the filter state after it (particles, random generator, recovery statistics).
/// Measurements are processed in the order of their stamps, whatever their arrival order: one
/// older than the newest is inserted at its time, the state before it is restored and the newer
/// measurements are replayed (re-filtering). A twist that arrives late does the same for the
/// interval it covers. The result is therefore the same as if everything had arrived in time
/// order, which the tests check bit for bit. Between measurements, buffered twists are integrated
/// piecewise; before the first twist the vehicle is assumed to stand still.
///
/// Sensors: any measurement is fused by update() through a MeasurementModel of it, so a new
/// sensor (camera landmarks, GNSS, ...) needs a model and an addSensor() call, no change here.
/// SensorParams sets per sensor how sharp its likelihood may be and whether it drives recovery.
/// Scans have a dedicated entry point because they share filtering, sampling and scan matching.
class Localizer
{
public:
  /// Built-in sensors.
  static constexpr SensorId kPointCloudSensor = 0;    // LocalizerParams::point_cloud_sensor
  static constexpr SensorId kPoseSensor = 1;          // LocalizerParams::pose_sensor
  static constexpr SensorId kScanMatchingSensor = 2;  // scan matching feedback, no recovery

  /// Throws std::invalid_argument for inconsistent parameters.
  explicit Localizer(const LocalizerParams & params);

  // Extension points: replace the models built from the parameters.
  void setMotionModel(std::unique_ptr<MotionModel> model);
  void setResampler(std::unique_ptr<Resampler> resampler);
  /// Enables refinement by scan matching (nullptr disables it).
  void setScanMatcher(std::unique_ptr<ScanMatcher> matcher) { scan_matcher_ = std::move(matcher); }
  /// Registers a sensor for update(). Throws std::invalid_argument for a negative threshold.
  SensorId addSensor(const SensorParams & params);
  /// Throws std::out_of_range for an unknown id.
  const SensorParams & sensor(SensorId id) const { return sensors_.at(id); }

  /// Sensor model of the scans of updateWithPointCloud(), which holds the map.
  /// Throws std::invalid_argument if it is null.
  void setPointCloudLikelihood(std::shared_ptr<const PointCloudLikelihood> likelihood);
  bool hasPointCloudLikelihood() const { return point_cloud_likelihood_ != nullptr; }

  /// (Re)starts the filter at `stamp`; clears the measurement history.
  void initialize(const PoseWithCovariance & initial_pose, const Time & stamp);
  /// Initializes from an arbitrary distribution, e.g. GroundPoseSampler for global localization.
  /// Prefer KLD resampling then, so that the particle count shrinks after convergence.
  void initialize(const PoseSampler & sampler, size_t particle_num, const Time & stamp);
  bool isInitialized() const { return initialized_; }

  /// Registers a body twist valid from `stamp` until the next one, in any order. A twist older
  /// than the newest measurement replays the measurements after it. Returns false if it is older
  /// than the history: it is then applied from the oldest kept state on only.
  bool addTwist(const Twist & twist, const Time & stamp);

  /// Moves the particles forward to `stamp` with the buffered twists (a measurement without
  /// correction, kept in the history like the others).
  LocalizerStatus predict(const Time & stamp);

  /// Fuses a measurement of `sensor` at `stamp`: predicts to it, corrects with `model`, then
  /// resamples (or resets) if needed, and replays newer measurements if it arrived late. Build the
  /// model first (validating the measurement), so that an invalid one leaves the filter untouched.
  /// Throws std::invalid_argument for an unknown sensor or a null model.
  LocalizerStatus update(
    SensorId sensor, std::shared_ptr<const MeasurementModel> model, const Time & stamp);

  /// Filters and samples a scan given in the base frame, then update() with the point cloud
  /// likelihood, then refines the estimate by scan matching if a ScanMatcher is set and the scan
  /// is the newest measurement. The sample is drawn from a generator seeded by the stamp, so that
  /// it does not depend on the arrival order.
  LocalizerStatus updateWithPointCloud(const PointCloud & scan, const Time & stamp);

  /// update() with an absolute pose observation (e.g. external scan matching, GNSS).
  LocalizerStatus updateWithPose(const PoseWithCovariance & observation, const Time & stamp);

  // State at the newest measurement.
  PoseWithCovariance estimate() const { return current().filter.estimate(); }
  /// Pose refined by scan matching of the newest measurement, if it is a scan and matched.
  std::optional<Pose> refinedPose() const;
  const ParticleSet & particles() const { return current().filter.particles(); }
  double effectiveSampleSize() const { return current().filter.effectiveSampleSize(); }
  /// Likelihood of the newest measurement update, for diagnostics.
  const LikelihoodSummary & lastLikelihood() const { return current().last_likelihood; }
  /// Number of expansion resets since initialization, for diagnostics.
  size_t expansionResetCount() const { return current().expansion_reset_count; }
  /// Time the particles currently represent (the newest measurement).
  const Time & stamp() const { return current().stamp; }
  /// Measurements kept for delayed ones, and measurements processed again so far (diagnostics).
  size_t historySize() const { return history_.size(); }
  size_t replayedCount() const { return replayed_count_; }

private:
  struct RecoveryStatistics
  {
    double w_slow = 0.0;
    double w_fast = 0.0;
  };

  /// Everything a measurement changes; restored to replay.
  struct FilterState
  {
    ParticleFilter filter;
    Time stamp;
    std::vector<RecoveryStatistics> recovery;  // per sensor
    LikelihoodSummary last_likelihood;
    size_t expansion_reset_count = 0;
  };

  struct Entry
  {
    Time stamp;
    SensorId sensor;
    std::shared_ptr<const MeasurementModel> model;  // null: prediction only
    FilterState state;                              // after this entry
  };

  const FilterState & current() const { return history_.empty() ? base_ : history_.back().state; }
  void initialize(ParticleFilter filter, const Time & stamp);
  LocalizerStatus insert(Entry entry);
  /// Recomputes the states of history_[index] and after.
  void replayFrom(size_t index);
  void prune();
  void process(FilterState & state, const Entry & entry) const;
  void predict(FilterState & state, const Time & stamp) const;
  void correct(FilterState & state, SensorId sensor, const MeasurementModel & model) const;
  std::vector<Eigen::Vector3d> samplePoints(
    const std::vector<Eigen::Vector3d> & points, const Time & stamp) const;
  void refineByScanMatching(const std::vector<Eigen::Vector3d> & points, const Time & stamp);

  LocalizerParams params_;
  std::vector<SensorParams> sensors_;
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<Resampler> resampler_;
  std::unique_ptr<ScanMatcher> scan_matcher_;
  GaussianPoseSampler injection_sampler_;
  GaussianPoseSampler expansion_sampler_;
  ScanFilter scan_filter_;
  std::shared_ptr<const PointCloudLikelihood> point_cloud_likelihood_;

  bool initialized_ = false;
  /// State before the oldest entry of history_ (older measurements are folded into it).
  FilterState base_;
  /// Time ordered (stable for equal stamps: arrival order).
  std::deque<Entry> history_;
  TwistHistory twists_;
  size_t replayed_count_ = 0;
  std::optional<std::pair<Time, Pose>> refined_pose_;
};

}  // namespace amcl_3d
