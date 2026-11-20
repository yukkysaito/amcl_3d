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
  base_{ParticleFilter(params.random_seed), Time{}, {}, {}, 0, 0, 0, {}}
{
  if (
    params_.scan_matching.feedback &&
    (params_.scan_matching.feedback_stddev.array() <= 0.0).any()) {
    throw std::invalid_argument("Localizer: scan_matching.feedback_stddev must be positive");
  }
  if (!(params_.max_measurement_delay >= 0.0)) {
    throw std::invalid_argument("Localizer: max_measurement_delay must not be negative");
  }
  addSensor(params_.point_cloud_sensor);
  addSensor(params_.pose_sensor);
  addSensor(SensorParams{"scan_matching", 0, false, 0.0});
}

SensorId Localizer::addSensor(const SensorParams & params)
{
  if (!(params.expansion_reset_threshold >= 0.0)) {
    throw std::invalid_argument("Localizer: expansion_reset_threshold must not be negative");
  }
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
  base_ = FilterState{std::move(filter), stamp, {}, {}, 0, ++last_generation_, 0, {}};
  history_.clear();
  refined_pose_.reset();
  initialized_ = true;
  twists_.pruneBefore(stamp);
}

// ---- inputs

bool Localizer::addTwist(const Twist & twist, const Time & stamp)
{
  twists_.insert(stamp, twist);
  if (!initialized_) {
    twists_.pruneBefore(Time{twists_.newestStamp().nanoseconds - kTwistBufferDurationNs});
    return true;
  }
  // Entries at `stamp` integrated up to it, so only later ones change.
  const auto first_affected = std::upper_bound(
    history_.begin(), history_.end(), stamp,
    [](const Time & t, const Entry & entry) { return t < entry.stamp; });
  if (first_affected != history_.end()) {
    const auto index = static_cast<size_t>(std::distance(history_.begin(), first_affected));
    replayed_count_ += history_.size() - index;
    replayFrom(index);
  }
  return !(stamp < base_.stamp);
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
  const Entry & inserted = *history_.insert(position, std::move(entry));
  const bool reweight = delayed && inserted.model &&
                        sensors_[inserted.sensor].delayed_fusion == DelayedFusion::kReweight;
  if (reweight && reweightFrom(index)) {
    ++reweighted_count_;
  } else {
    replayed_count_ += history_.size() - index - 1;
    replayFrom(index);
  }
  prune();
  return LocalizerStatus::kOk;
}

void Localizer::replayFrom(size_t index)
{
  FilterState state = index == 0 ? base_ : history_[index - 1].state;
  for (size_t i = index; i < history_.size(); ++i) {
    advance(state, history_[i]);
    if (i + 1 < history_.size()) {
      history_[i].state = state;
    } else {
      history_[i].state = std::move(state);
    }
  }
}

bool Localizer::reweightFrom(size_t index)
{
  const FilterState & before = index == 0 ? base_ : history_[index - 1].state;
  Entry & entry = history_[index];
  const FilterState & after = history_[index + 1].state;  // exists: the entry is delayed
  if (after.parent_generation != before.generation) {
    return false;  // computed from another branch (a measurement reweighted earlier)
  }

  // Ancestor among the particles of `after` of every particle of the newer states.
  std::vector<std::pair<uint64_t, std::vector<size_t>>> ancestors;
  ancestors.reserve(history_.size() - index - 1);
  {
    std::vector<size_t> identity(after.filter.particles().size());
    std::iota(identity.begin(), identity.end(), 0);
    ancestors.emplace_back(after.generation, std::move(identity));
  }
  for (size_t i = index + 2; i < history_.size(); ++i) {
    const FilterState & state = history_[i].state;
    const auto parent = std::find_if(ancestors.rbegin(), ancestors.rend(), [&](const auto & a) {
      return a.first == state.parent_generation;
    });
    if (parent == ancestors.rend()) {
      return false;
    }
    std::vector<size_t> of_state(state.filter.particles().size());
    for (size_t j = 0; j < of_state.size(); ++j) {
      of_state[j] = parent->second.at(state.parents.empty() ? j : state.parents[j]);
    }
    ancestors.emplace_back(state.generation, std::move(of_state));
  }

  // Every particle of `after` was predicted from its parent in `before`: its pose at the
  // measurement time lies on that path. Evaluating new predictions instead would score other
  // random paths, which a sharp likelihood (a scan) does not correlate with the actual ones.
  const ParticleSet & from = before.filter.particles();
  const ParticleSet & to = after.filter.particles();
  const double span = static_cast<double>(after.stamp.nanoseconds - before.stamp.nanoseconds);
  const double ratio =
    span > 0.0 ? static_cast<double>(entry.stamp.nanoseconds - before.stamp.nanoseconds) / span
               : 1.0;
  ParticleSet at_measurement(to.size());
  for (size_t j = 0; j < to.size(); ++j) {
    const Pose a = from.pose(after.parents.empty() ? j : after.parents[j]);
    const Pose b = to.pose(j);
    Pose pose;
    pose.position = a.position + ratio * (b.position - a.position);
    pose.orientation = a.orientation.slerp(ratio, b.orientation);
    at_measurement.setPose(j, pose);
  }
  const LogLikelihood log_likelihood =
    at_measurement.empty() ? LogLikelihood{} : entry.model->computeLogLikelihood(at_measurement);
  const Eigen::VectorXd increments =
    temperingExponent(
      log_likelihood.observation_num, sensors_[entry.sensor].max_effective_observation_num) *
    log_likelihood.values;

  // The state after the measurement itself is computed exactly, so that a replay from it is.
  FilterState state = before;
  advance(state, entry);
  entry.state = std::move(state);

  // The newer states inherit the likelihood of their ancestors.
  for (size_t i = index + 1; i < history_.size(); ++i) {
    const std::vector<size_t> & of_state = ancestors[i - index - 1].second;
    Eigen::VectorXd state_increments(static_cast<Eigen::Index>(of_state.size()));
    for (size_t j = 0; j < of_state.size(); ++j) {
      state_increments(static_cast<Eigen::Index>(j)) =
        increments(static_cast<Eigen::Index>(of_state[j]));
    }
    history_[i].state.filter.reweight(state_increments);
  }
  FilterState & newest = history_.back().state;
  if (needsResampling(newest.filter)) {
    const std::vector<size_t> parents = newest.filter.resample(*resampler_);
    if (!newest.parents.empty()) {
      std::vector<size_t> composed(parents.size());
      for (size_t j = 0; j < parents.size(); ++j) {
        composed[j] = newest.parents[parents[j]];
      }
      newest.parents = std::move(composed);
    } else {
      newest.parents = parents;
    }
  }
  return true;
}

void Localizer::advance(FilterState & state, const Entry & entry)
{
  state.parents.clear();
  state.parent_generation = state.generation;
  process(state, entry);
  state.generation = ++last_generation_;
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
    const ParticleSet & particles = state.filter.particles();
    correct(
      state, entry.sensor,
      particles.empty() ? LogLikelihood{} : entry.model->computeLogLikelihood(particles));
  }
}

void Localizer::predict(FilterState & state, const Time & stamp) const
{
  twists_.integrate(state.stamp, stamp, [&](const Twist & twist, double dt) {
    state.filter.predict(*motion_model_, twist, dt);
  });
  state.stamp = stamp;
}

void Localizer::correct(
  FilterState & state, SensorId sensor, const LogLikelihood & log_likelihood) const
{
  const SensorParams & sensor_params = sensors_[sensor];
  state.last_likelihood =
    state.filter.update(log_likelihood, sensor_params.max_effective_observation_num);

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
  state.parents = state.filter.resample(*resampler_);
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
  if (params_.scan_matching.feedback) {
    PoseWithCovariance observation;
    observation.pose = result.pose;
    observation.covariance = params_.scan_matching.feedback_stddev.cwiseAbs2().asDiagonal();
    update(kScanMatchingSensor, std::make_shared<PoseMeasurementModel>(observation), stamp);
  }
  refined_pose_.emplace(stamp, result.pose);
}

}  // namespace amcl_3d
