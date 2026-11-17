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
    GaussianPoseSampler::fromStandardDeviations(params.augmented_mcl.injection_stddev))
{
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

void Localizer::initialize(const PoseWithCovariance & initial_pose, const Time & stamp)
{
  filter_.initialize(initial_pose, params_.initial_particle_num);
  stamp_ = stamp;
  w_slow_ = 0.0;
  w_fast_ = 0.0;
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
  std::vector<Eigen::Vector3d> points = selectScanPoints(scan);
  if (points.empty()) {
    return LocalizerStatus::kInvalidMeasurement;
  }
  const LocalizerStatus status = predict(stamp);
  if (status != LocalizerStatus::kOk) {
    return status;
  }
  correct(PointCloudMeasurementModel(map_, std::move(points), params_.point_cloud_measurement));
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
  const double average_likelihood = filter_.update(model);
  w_slow_ += params_.augmented_mcl.alpha_slow * (average_likelihood - w_slow_);
  w_fast_ += params_.augmented_mcl.alpha_fast * (average_likelihood - w_fast_);

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

std::vector<Eigen::Vector3d> Localizer::selectScanPoints(const PointCloud & scan)
{
  std::vector<Eigen::Vector3d> finite_points;
  finite_points.reserve(scan.size());
  for (const auto & point : scan) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite_points.emplace_back(point.x, point.y, point.z);
    }
  }
  const size_t sample_num = params_.scan_sample_num;
  if (sample_num == 0 || finite_points.size() <= sample_num) {
    return finite_points;
  }
  std::vector<Eigen::Vector3d> sampled;
  sampled.reserve(sample_num);
  std::sample(
    finite_points.begin(), finite_points.end(), std::back_inserter(sampled), sample_num,
    filter_.randomGenerator());
  return sampled;
}

}  // namespace amcl_3d
