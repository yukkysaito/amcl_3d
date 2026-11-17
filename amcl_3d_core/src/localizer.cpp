#include "amcl_3d_core/localizer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <stdexcept>

#include "amcl_3d_core/measurement_model/pose_measurement_model.hpp"
#include "amcl_3d_core/resampler/systematic_resampler.hpp"

namespace amcl_3d
{
namespace
{
/// While not initialized, twists older than this relative to the newest one are dropped.
constexpr int64_t kTwistBufferDurationNs = 10'000'000'000;  // 10 s

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
      return "stale time stamp";
    case LocalizerStatus::kInvalidMeasurement:
      return "invalid measurement";
  }
  return "unknown";
}

Localizer::Localizer(const LocalizerParams & params)
: params_(params),
  filter_(params.random_seed),
  motion_model_(std::make_unique<TwistMotionModel>(params.motion_model)),
  resampler_(createResampler(params.resampling)),
  injection_sampler_(
    GaussianPoseSampler::fromStandardDeviations(params.augmented_mcl.injection_stddev)),
  expansion_sampler_(GaussianPoseSampler::fromStandardDeviations(params.expansion_reset.stddev)),
  scan_filter_(params.scan_filter)
{
  if (
    params_.scan_matching.feedback &&
    (params_.scan_matching.feedback_stddev.array() <= 0.0).any()) {
    throw std::invalid_argument("Localizer: scan_matching.feedback_stddev must be positive");
  }
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

void Localizer::setMap(std::shared_ptr<const PointCloudMap> map)
{
  if (!map) {
    throw std::invalid_argument("Localizer: map is null");
  }
  if (map->maxDistance() < params_.point_cloud_measurement.max_distance) {
    throw std::invalid_argument(
      "Localizer: the map distance range is shorter than point_cloud_measurement.max_distance");
  }
  map_ = std::move(map);
}

void Localizer::initialize(const PoseWithCovariance & initial_pose, const Time & stamp)
{
  filter_.initialize(initial_pose, params_.initial_particle_num);
  resetState(stamp);
}

void Localizer::initialize(const PoseSampler & sampler, size_t particle_num, const Time & stamp)
{
  filter_.initialize(sampler, particle_num);
  resetState(stamp);
}

void Localizer::resetState(const Time & stamp)
{
  stamp_ = stamp;
  w_slow_ = 0.0;
  w_fast_ = 0.0;
  last_likelihood_ = LikelihoodSummary{};
  expansion_reset_count_ = 0;
  refined_pose_.reset();
  initialized_ = true;
  pruneTwists();
}

bool Localizer::addTwist(const Twist & twist, const Time & stamp)
{
  if (!twists_.empty() && stamp < twists_.back().first) {
    return false;
  }
  twists_.emplace_back(stamp, twist);
  pruneTwists();
  return true;
}

void Localizer::pruneTwists()
{
  // Keep the twist active at the reference time and everything after it.
  if (twists_.empty()) {
    return;
  }
  const Time reference =
    initialized_ ? stamp_ : Time{twists_.back().first.nanoseconds - kTwistBufferDurationNs};
  while (twists_.size() >= 2 && twists_[1].first <= reference) {
    twists_.pop_front();
  }
}

LocalizerStatus Localizer::predict(const Time & stamp)
{
  if (!initialized_) {
    return LocalizerStatus::kNotInitialized;
  }
  if (stamp < stamp_) {
    return LocalizerStatus::kStaleStamp;
  }
  // Integrate piecewise constant twists. Before the first twist, the vehicle is assumed to stop.
  while (stamp_ < stamp) {
    Twist active_twist;
    Time segment_end = stamp;
    if (!twists_.empty()) {
      if (twists_.front().first <= stamp_) {
        active_twist = twists_.front().second;
        if (twists_.size() >= 2) {
          segment_end = std::min(stamp, twists_[1].first);
        }
      } else {
        segment_end = std::min(stamp, twists_.front().first);
      }
    }
    filter_.predict(*motion_model_, active_twist, segment_end - stamp_);
    stamp_ = segment_end;
    pruneTwists();
  }
  return LocalizerStatus::kOk;
}

// Measurements are validated before predict() so that a rejected one leaves the filter untouched.

LocalizerStatus Localizer::updateWithPointCloud(const PointCloud & scan, const Time & stamp)
{
  if (!hasMap()) {
    return LocalizerStatus::kNoMap;
  }
  const std::vector<Eigen::Vector3d> points = scan_filter_.apply(scan);
  if (points.empty()) {
    return LocalizerStatus::kInvalidMeasurement;
  }
  const LocalizerStatus status = predict(stamp);
  if (status != LocalizerStatus::kOk) {
    return status;
  }
  correct(PointCloudMeasurementModel(map_, samplePoints(points), params_.point_cloud_measurement));
  if (scan_matcher_) {
    refineByScanMatching(points);
  }
  return LocalizerStatus::kOk;
}

LocalizerStatus Localizer::updateWithPose(
  const PoseWithCovariance & observation, const Time & stamp)
{
  std::optional<PoseMeasurementModel> model;
  try {
    model.emplace(observation);
  } catch (const std::invalid_argument &) {
    return LocalizerStatus::kInvalidMeasurement;
  }
  const LocalizerStatus status = predict(stamp);
  if (status != LocalizerStatus::kOk) {
    return status;
  }
  correct(*model);
  return LocalizerStatus::kOk;
}

void Localizer::correct(const MeasurementModel & model)
{
  last_likelihood_ = filter_.update(model);
  w_slow_ += params_.augmented_mcl.alpha_slow * (last_likelihood_.mean - w_slow_);
  w_fast_ += params_.augmented_mcl.alpha_fast * (last_likelihood_.mean - w_fast_);

  if (
    params_.expansion_reset.enabled &&
    last_likelihood_.max < params_.expansion_reset.likelihood_threshold) {
    filter_.expand(expansion_sampler_);
    ++expansion_reset_count_;
    return;
  }

  const double particle_num = static_cast<double>(filter_.particles().size());
  const double threshold = params_.resampling.ess_ratio_threshold;
  if (threshold < 1.0 && filter_.effectiveSampleSize() >= threshold * particle_num) {
    return;
  }
  filter_.resample(*resampler_);
  if (params_.augmented_mcl.enabled && w_slow_ > 0.0) {
    filter_.injectRandomParticles(std::max(0.0, 1.0 - w_fast_ / w_slow_), injection_sampler_);
  }
}

std::vector<Eigen::Vector3d> Localizer::samplePoints(const std::vector<Eigen::Vector3d> & points)
{
  const size_t sample_num = params_.scan_sample_num;
  if (sample_num == 0 || points.size() <= sample_num) {
    return points;
  }
  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve(sample_num);
  std::sample(
    points.begin(), points.end(), std::back_inserter(sampled), sample_num,
    filter_.randomGenerator());
  return sampled;
}

void Localizer::refineByScanMatching(const std::vector<Eigen::Vector3d> & points)
{
  refined_pose_.reset();
  const Pose estimate = filter_.estimate().pose;
  const ScanMatchingResult result = scan_matcher_->align(points, estimate);
  const double correction = (result.pose.position - estimate.position).norm();
  if (!result.converged || correction > params_.scan_matching.max_correction) {
    return;
  }
  refined_pose_ = result.pose;
  if (params_.scan_matching.feedback) {
    PoseWithCovariance observation;
    observation.pose = result.pose;
    observation.covariance = params_.scan_matching.feedback_stddev.cwiseAbs2().asDiagonal();
    correct(PoseMeasurementModel(observation));
  }
}

}  // namespace amcl_3d
