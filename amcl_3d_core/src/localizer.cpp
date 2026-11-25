#include "amcl_3d_core/localizer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <optional>
#include <stdexcept>

#include "amcl_3d_core/measurement_model/pose_measurement_model.hpp"
#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/resampler/systematic_resampler.hpp"
#include "amcl_3d_core/update/update_factory.hpp"

namespace amcl_3d
{
namespace
{
/// While not initialized, twists older than this relative to the newest one are dropped.
constexpr int64_t kTwistBufferDurationNs = 10'000'000'000;  // 10 s
/// Separates the scan sampling streams from the other users of the seed.
constexpr uint64_t kSamplingSeedSalt = 0x5CA4'5A3B'1E5E'ED01ULL;

std::unique_ptr<Resampler> createResampler(const LocalizerParams::Resampling & params)
{
  switch (params.type) {
    case ResamplerType::kKld:
      return std::make_unique<KldResampler>(params.kld);
    case ResamplerType::kSystematic:
      return std::make_unique<SystematicResampler>();
  }
  throw std::invalid_argument("unknown resampler type");
}

bool isFinite(const PoseWithCovariance & pose)
{
  return pose.pose.position.allFinite() && pose.pose.orientation.coeffs().allFinite() &&
         pose.covariance.allFinite();
}
}  // namespace

const char * toString(LocalizerStatus status)
{
  switch (status) {
    case LocalizerStatus::kOk:
      return "ok";
    case LocalizerStatus::kNotInitialized:
      return "not initialized";
    case LocalizerStatus::kNoMap:
      return "no map";
    case LocalizerStatus::kStaleStamp:
      return "older than max_measurement_delay";
    case LocalizerStatus::kInvalidMeasurement:
      return "invalid measurement";
  }
  return "unknown";
}

PoseWithCovariance Localizer::estimate() const
{
  const ParticleFilter & filter = current().filter;
  switch (params_.estimate.type) {
    case EstimateType::kBestCluster:
      return filter.bestClusterEstimate(params_.estimate.clustering);
    case EstimateType::kMean:
      break;
  }
  return filter.estimate();
}

Localizer::Localizer(const LocalizerParams & params)
: params_(params),
  motion_model_(createMotionModel(params.motion_model)),
  resampler_(createResampler(params.resampling)),
  injection_sampler_(
    GaussianPoseSampler::fromStandardDeviations(params.augmented_mcl.injection_stddev)),
  expansion_sampler_(GaussianPoseSampler::fromStandardDeviations(params.expansion_reset.stddev)),
  scan_filter_(params.scan_filter),
  base_{ParticleFilter(params.random_seed), Time{}, {}, {}, 0, {}}
{
  if (!(params_.max_measurement_delay >= 0.0)) {
    throw std::invalid_argument("Localizer: max_measurement_delay must not be negative");
  }
  addSensor(params_.point_cloud_sensor);
  addSensor(params_.pose_sensor);
}

SensorId Localizer::addSensor(const SensorParams & params)
{
  if (!(params.expansion_reset_threshold >= 0.0)) {
    throw std::invalid_argument("Localizer: expansion_reset_threshold must not be negative");
  }
  updates_.push_back(createMeasurementUpdate(params));
  sensors_.push_back(params);
  return sensors_.size() - 1;
}

void Localizer::setMotionModel(std::unique_ptr<MotionModel> model)
{
  if (!model) {
    throw std::invalid_argument("Localizer: motion model is null");
  }
  motion_model_ = std::move(model);
}

void Localizer::setResampler(std::unique_ptr<Resampler> resampler)
{
  if (!resampler) {
    throw std::invalid_argument("Localizer: resampler is null");
  }
  resampler_ = std::move(resampler);
}

void Localizer::setPointCloudLikelihood(std::shared_ptr<const PointCloudLikelihood> likelihood)
{
  if (!likelihood) {
    throw std::invalid_argument("Localizer: point cloud likelihood is null");
  }
  point_cloud_likelihood_ = std::move(likelihood);
}

// ---- initialization

void Localizer::initialize(const PoseWithCovariance & initial_pose, const Time & stamp)
{
  if (!isFinite(initial_pose)) {
    throw std::invalid_argument("Localizer: the initial pose is not finite");
  }
  ParticleFilter filter = current().filter;  // continues the random sequence
  filter.initialize(initial_pose, params_.initial_particle_num);
  initialize(std::move(filter), stamp);
}

void Localizer::initialize(const PoseSampler & sampler, size_t particle_num, const Time & stamp)
{
  ParticleFilter filter = current().filter;
  filter.initialize(sampler, particle_num);
  initialize(std::move(filter), stamp);
}

void Localizer::initialize(ParticleFilter filter, const Time & stamp)
{
  base_ = FilterState{std::move(filter), stamp, {}, {}, 0, {}};
  history_.clear();
  refined_pose_.reset();
  initialized_ = true;
  twists_.pruneBefore(stamp);
}

// ---- inputs

LocalizerStatus Localizer::addTwist(const Twist & twist, const Time & stamp)
{
  // one NaN would spread to every particle and stay there until re-initialization
  if (!twist.linear.allFinite() || !twist.angular.allFinite()) {
    return LocalizerStatus::kInvalidMeasurement;
  }
  if (!initialized_) {
    twists_.insert(stamp, twist);
    twists_.pruneBefore(Time{twists_.newestStamp().nanoseconds - kTwistBufferDurationNs});
    return LocalizerStatus::kOk;
  }
  if (stamp < base_.stamp && twists_.startsWithin(stamp, base_.stamp)) {
    return LocalizerStatus::kStaleStamp;
  }
  twists_.insert(stamp, twist);
  // Entries at `stamp` integrated up to it, so only later ones change.
  const auto first_affected = std::upper_bound(
    history_.begin(), history_.end(), stamp,
    [](const Time & t, const Entry & entry) { return t < entry.stamp; });
  if (first_affected != history_.end()) {
    const auto index = static_cast<size_t>(std::distance(history_.begin(), first_affected));
    replayed_count_ += history_.size() - index;
    replayFrom(index);
  }
  // Without measurements, e.g. while the LiDAR is down, the history does not advance and the
  // twists after it pile up. Predict up to the history length before the newest twist, so that
  // late measurements still fit.
  const int64_t max_delay_ns =
    static_cast<int64_t>(std::llround(params_.max_measurement_delay * 1e9));
  const Time newest = twists_.newestStamp();
  if (
    newest.nanoseconds - this->stamp().nanoseconds >
    std::max(kTwistBufferDurationNs, 2 * max_delay_ns)) {
    predict(Time{newest.nanoseconds - max_delay_ns});
  }
  return LocalizerStatus::kOk;
}

LocalizerStatus Localizer::predict(const Time & stamp)
{
  return insert(Entry{stamp, kPoseSensor, nullptr, {}});
}

LocalizerStatus Localizer::update(
  SensorId sensor, std::shared_ptr<const MeasurementModel> model, const Time & stamp)
{
  if (sensor >= sensors_.size()) {
    throw std::invalid_argument("Localizer: unknown sensor");
  }
  if (!model) {
    throw std::invalid_argument("Localizer: measurement model is null");
  }
  return insert(Entry{stamp, sensor, std::move(model), {}});
}

// Measurements are validated before they are inserted, so that a rejected one leaves the filter
// untouched.

LocalizerStatus Localizer::updateWithPointCloud(const PointCloud & scan, const Time & stamp)
{
  if (!hasPointCloudLikelihood()) {
    return LocalizerStatus::kNoMap;
  }
  const std::vector<Eigen::Vector3d> points = scan_filter_.apply(scan);
  if (points.empty()) {
    return LocalizerStatus::kInvalidMeasurement;
  }
  if (!initialized_) {
    return LocalizerStatus::kNotInitialized;
  }
  if (stamp < base_.stamp) {
    return LocalizerStatus::kStaleStamp;  // before building the model, which may be costly
  }
  const LocalizerStatus status = update(
    kPointCloudSensor,
    point_cloud_likelihood_->createMeasurementModel(points, samplePoints(points, stamp)), stamp);
  if (status == LocalizerStatus::kOk && scan_matcher_ && stamp == this->stamp()) {
    refineByScanMatching(points, stamp);
  }
  return status;
}

LocalizerStatus Localizer::updateWithPose(
  const PoseWithCovariance & observation, const Time & stamp)
{
  std::shared_ptr<const MeasurementModel> model;
  try {
    model = std::make_shared<PoseMeasurementModel>(observation);
  } catch (const std::invalid_argument &) {
    return LocalizerStatus::kInvalidMeasurement;
  }
  return update(kPoseSensor, std::move(model), stamp);
}

std::optional<Pose> Localizer::refinedPose() const
{
  if (!refined_pose_ || !(refined_pose_->first == stamp())) {
    return std::nullopt;
  }
  return refined_pose_->second;
}

// ---- history

LocalizerStatus Localizer::insert(Entry entry)
{
  if (!initialized_) {
    return LocalizerStatus::kNotInitialized;
  }
  if (entry.stamp < base_.stamp) {
    return LocalizerStatus::kStaleStamp;
  }
  // after the entries with the same stamp: equal stamps keep the arrival order
  const auto position = std::upper_bound(
    history_.begin(), history_.end(), entry.stamp,
    [](const Time & t, const Entry & e) { return t < e.stamp; });
  const auto index = static_cast<size_t>(std::distance(history_.begin(), position));
  const bool delayed = position != history_.end();
  history_.insert(position, std::move(entry));
  if (delayed) {
    replayed_count_ += history_.size() - index - 1;
  }
  replayFrom(index);
  prune();
  return LocalizerStatus::kOk;
}

void Localizer::replayFrom(size_t index)
{
  FilterState state = index == 0 ? base_ : history_[index - 1].state;
  for (size_t i = index; i < history_.size(); ++i) {
    process(state, history_[i]);
    if (i + 1 < history_.size()) {
      history_[i].state = state;
    } else {
      history_[i].state = std::move(state);
    }
  }
}

void Localizer::prune()
{
  // Fold the entries no delayed measurement can precede any more into the base state.
  const Time horizon{
    stamp().nanoseconds - static_cast<int64_t>(std::llround(params_.max_measurement_delay * 1e9))};
  while (!history_.empty() && history_.front().stamp <= horizon) {
    base_ = std::move(history_.front().state);
    history_.pop_front();
  }
  twists_.pruneBefore(base_.stamp);
}

// ---- filter steps (const: they only change the given state)

void Localizer::process(FilterState & state, const Entry & entry) const
{
  predict(state, entry.stamp);
  if (entry.model) {
    correct(state, entry.sensor, *entry.model);
    state.filter.startTransitionPrior();
  }
}

void Localizer::predict(FilterState & state, const Time & stamp) const
{
  twists_.integrate(state.stamp, stamp, [&](const Twist & twist, double dt) {
    state.filter.predict(*motion_model_, twist, dt);
  });
  state.stamp = stamp;
}

void Localizer::correct(FilterState & state, SensorId sensor, const MeasurementModel & model) const
{
  const SensorParams & sensor_params = sensors_[sensor];
  UpdateContext context;
  context.max_effective_observation_num = sensor_params.max_effective_observation_num;
  context.ess_ratio_threshold = params_.resampling.ess_ratio_threshold;
  context.resample = [this, &state]() { return state.filter.resample(*resampler_); };
  context.clustering = params_.estimate.clustering;
  const UpdateResult result = updates_[sensor]->apply(state.filter, model, context);
  state.last_likelihood = result.likelihood;
  state.last_update = result.statistics;

  if (sensor_params.drives_recovery) {
    if (state.recovery.size() <= sensor) {
      state.recovery.resize(sensor + 1);
    }
    RecoveryStatistics & statistics = state.recovery[sensor];
    const double mean = state.last_likelihood.mean;
    statistics.w_slow += params_.augmented_mcl.alpha_slow * (mean - statistics.w_slow);
    statistics.w_fast += params_.augmented_mcl.alpha_fast * (mean - statistics.w_fast);

    if (
      params_.expansion_reset.enabled &&
      state.last_likelihood.max < sensor_params.expansion_reset_threshold) {
      state.filter.expand(expansion_sampler_);
      ++state.expansion_reset_count;
      return;
    }
  }

  if (!needsResampling(state.filter)) {
    return;
  }
  state.filter.resample(*resampler_);
  if (params_.augmented_mcl.enabled) {
    double ratio = 0.0;
    for (const auto & statistics : state.recovery) {
      if (statistics.w_slow > 0.0) {
        ratio = std::max(ratio, 1.0 - statistics.w_fast / statistics.w_slow);
      }
    }
    if (ratio > 0.0) {
      state.filter.injectRandomParticles(ratio, injection_sampler_);
    }
  }
}

bool Localizer::needsResampling(const ParticleFilter & filter) const
{
  const double threshold = params_.resampling.ess_ratio_threshold;
  return threshold >= 1.0 ||
         filter.effectiveSampleSize() < threshold * static_cast<double>(filter.particles().size());
}

// ---- scans

std::vector<Eigen::Vector3d> Localizer::samplePoints(
  const std::vector<Eigen::Vector3d> & points, const Time & stamp) const
{
  const size_t sample_num = params_.scan_sample_num;
  if (sample_num == 0 || points.size() <= sample_num) {
    return points;
  }
  RandomGenerator rng = parallelStream(
    params_.random_seed ^ kSamplingSeedSalt, static_cast<uint64_t>(stamp.nanoseconds));
  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve(sample_num);
  std::sample(points.begin(), points.end(), std::back_inserter(sampled), sample_num, rng);
  return sampled;
}

void Localizer::refineByScanMatching(
  const std::vector<Eigen::Vector3d> & points, const Time & stamp)
{
  refined_pose_.reset();
  const Pose estimate = this->estimate().pose;
  const ScanMatchingResult result = scan_matcher_->align(points, estimate);
  const double correction = (result.pose.position - estimate.position).norm();
  if (!result.converged || correction > params_.scan_matching.max_correction) {
    return;
  }
  refined_pose_.emplace(stamp, result.pose);
}

}  // namespace amcl_3d
