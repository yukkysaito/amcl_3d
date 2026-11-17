#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"
#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/resampler/kld_resampler.hpp"
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
  /// Scan points randomly selected per update. 0 uses all points.
  size_t scan_sample_num = 200;
  TwistMotionModel::Params motion_model;
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
  explicit Localizer(const LocalizerParams & params);

  // Extension points: replace the models built from the parameters.
  void setMotionModel(std::unique_ptr<MotionModel> model);
  void setResampler(std::unique_ptr<Resampler> resampler);

  void setMap(std::shared_ptr<const PointCloudMap> map) { map_ = std::move(map); }
  bool hasMap() const { return map_ != nullptr; }

  void initialize(const PoseWithCovariance & initial_pose, const Time & stamp);
  bool isInitialized() const { return initialized_; }

  /// Registers a body twist valid from `stamp` until the next one. Twists must be added in time
  /// order; returns false (and ignores the twist) otherwise.
  bool addTwist(const Twist & twist, const Time & stamp);

  /// Moves the particles forward to `stamp` with the buffered twists.
  LocalizerStatus predict(const Time & stamp);

  /// predict(stamp), then corrects with a scan given in the base frame, then resamples if needed.
  LocalizerStatus updateWithPointCloud(const PointCloud & scan, const Time & stamp);

  /// predict(stamp), then corrects with an absolute pose observation (e.g. NDT, GNSS).
  LocalizerStatus updateWithPose(const PoseWithCovariance & observation, const Time & stamp);

  PoseWithCovariance estimate() const { return filter_.estimate(); }
  const ParticleSet & particles() const { return filter_.particles(); }
  double effectiveSampleSize() const { return filter_.effectiveSampleSize(); }
  /// Time the particles currently represent.
  const Time & stamp() const { return stamp_; }

private:
  void correct(const MeasurementModel & model);
  std::vector<Eigen::Vector3d> selectScanPoints(const PointCloud & scan);
  void pruneTwists();

  LocalizerParams params_;
  ParticleFilter filter_;
  std::unique_ptr<MotionModel> motion_model_;
  std::unique_ptr<Resampler> resampler_;
  GaussianPoseSampler injection_sampler_;
  std::shared_ptr<const PointCloudMap> map_;

  bool initialized_ = false;
  Time stamp_;
  /// Time ordered. After pruning, at most the front entry is at or before stamp_ (the active one).
  std::deque<std::pair<Time, Twist>> twists_;
  double w_slow_ = 0.0;
  double w_fast_ = 0.0;
};

}  // namespace amcl_3d
