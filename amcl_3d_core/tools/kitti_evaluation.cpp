// Evaluates the localizer on a KITTI odometry sequence against the ground truth.
//
//   kitti_evaluation <dataset_dir> <sequence> [key=value ...]
//
// <dataset_dir> contains sequences/<seq>/{calib.txt,times.txt,velodyne/*.bin} and poses/<seq>.txt.
//
// Protocol (see README):
// - The map is built from the even frames registered with the ground truth poses.
// - The odd frames are localized (5 Hz). Map and localized scans are different scans of the same
//   drive, so this measures accuracy on real LiDAR data, not robustness to map changes.
// - KITTI odometry has no wheel odometry: the twist is derived from the ground truth and
//   corrupted with a scale error, a yaw rate bias and white noise. Dead reckoning with the same
//   twist is reported as a baseline.
// - The filter is initialized off the ground truth by `init_offset` [m] and `init_yaw_offset`.
//
// Options (defaults in Config): map=distance_field|kdtree device=cpu|cuda resolution=<m>
// threads=<n> resampler=systematic|kld particles=<n> sample=<n> scan_matching=0|1
// expansion_reset=0|1 seed=<n> frames=<n> output=<trajectory.csv>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/map_factory.hpp"
#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/scan_filter.hpp"
#include "amcl_3d_core/scan_matcher/ndt_scan_matcher.hpp"
#include "kitti_dataset.hpp"
#include "statistics.hpp"

using namespace amcl_3d;
using namespace evaluation;
using kitti::loadScan;

namespace
{
// ---------------------------------------------------------------------------------------------
// configuration

struct Config
{
  std::string map_type = "distance_field";
  std::string device = "cpu";  // cuda: likelihood of the distance field on the GPU
  double map_resolution = 0.2;
  size_t threads = 4;  // likelihood threads, 0: all
  std::string resampler = "systematic";
  size_t particles = 500;
  size_t sample = 200;
  bool scan_matching = false;
  bool expansion_reset = false;
  uint64_t seed = 1;
  size_t frames = 0;  // 0: all
  std::string output;
  // odometry corruption
  double odom_scale_error = 0.02;
  double odom_yaw_rate_bias = 0.005;  // [rad/s]
  double odom_linear_noise = 0.1;     // [m/s]
  double odom_angular_noise = 0.01;   // [rad/s]
  double init_offset = 0.5;           // [m]
  double init_yaw_offset = 0.03;      // [rad]
  size_t warmup_frames = 10;          // excluded from the statistics
};

Config parseConfig(int argc, char ** argv)
{
  Config c;
  std::map<std::string, std::string> kv;
  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto eq = arg.find('=');
    if (eq == std::string::npos) {
      throw std::invalid_argument("expected key=value: " + arg);
    }
    kv[arg.substr(0, eq)] = arg.substr(eq + 1);
  }
  auto take = [&kv](const std::string & key, auto & value) {
    const auto it = kv.find(key);
    if (it == kv.end()) {
      return;
    }
    std::istringstream stream(it->second);
    stream >> value;
    kv.erase(it);
  };
  take("map", c.map_type);
  take("resolution", c.map_resolution);
  take("threads", c.threads);
  take("device", c.device);
  take("resampler", c.resampler);
  take("particles", c.particles);
  take("sample", c.sample);
  take("scan_matching", c.scan_matching);
  take("expansion_reset", c.expansion_reset);
  take("seed", c.seed);
  take("frames", c.frames);
  take("output", c.output);
  if (!kv.empty()) {
    throw std::invalid_argument("unknown option: " + kv.begin()->first);
  }
  return c;
}

PointCloud toCloud(const std::vector<Eigen::Vector3d> & points, const Pose & pose)
{
  PointCloud cloud;
  cloud.reserve(points.size());
  for (const auto & p : points) {
    const Eigen::Vector3d q = pose.orientation * p + pose.position;
    cloud.push_back(pcl::PointXYZ(
      static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())));
  }
  return cloud;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: kitti_evaluation <dataset_dir> <sequence> [key=value ...]\n";
    return 1;
  }
  const Config config = parseConfig(argc, argv);
  const kitti::Sequence sequence = kitti::loadSequence(argv[1], argv[2]);
  const size_t frame_num =
    config.frames == 0 ? sequence.poses.size() : std::min(config.frames, sequence.poses.size());

  // ---- map from the even frames ----
  const auto map_start = std::chrono::steady_clock::now();
  ScanFilter::Params map_scan_filter;
  map_scan_filter.min_range = 3.0;  // ego vehicle
  map_scan_filter.max_range = 80.0;
  map_scan_filter.voxel_size = 0.2;
  PointCloud accumulated;
  for (size_t k = 0; k < frame_num; k += 2) {
    accumulated +=
      toCloud(ScanFilter(map_scan_filter).apply(loadScan(sequence, k)), sequence.poses[k]);
  }
  ScanFilter::Params map_voxel;
  map_voxel.voxel_size = 0.2;
  const PointCloud::Ptr map(
    new PointCloud(toCloud(ScanFilter(map_voxel).apply(accumulated), Pose{})));

  // ---- localizer ----
  LocalizerParams params;
  params.random_seed = config.seed;
  params.initial_particle_num = config.particles;
  params.scan_sample_num = config.sample;
  params.scan_filter.min_range = 3.0;
  params.scan_filter.max_range = 80.0;
  params.scan_filter.voxel_size = 0.5;
  params.resampling.type =
    config.resampler == "kld" ? ResamplerType::kKld : ResamplerType::kSystematic;
  params.expansion_reset.enabled = config.expansion_reset;
  params.point_cloud_measurement.thread_num = config.threads;
  Localizer localizer(params);
  MapParams map_params;
  map_params.type = toMapType(config.map_type);
  map_params.device = toMapDevice(config.device);
  map_params.resolution = config.map_resolution;
  const auto point_cloud_map =
    createPointCloudMap(map, map_params, params.point_cloud_measurement.max_distance);
  localizer.setMap(point_cloud_map);
  if (config.scan_matching) {
    localizer.setScanMatcher(std::make_unique<NdtScanMatcher>(map, NdtScanMatcher::Params{}));
  }
  const double map_seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - map_start).count();

  PoseWithCovariance initial;
  initial.pose = sequence.poses[1];
  initial.pose.position.x() += config.init_offset;
  initial.pose.orientation =
    expMap(Eigen::Vector3d(0, 0, config.init_yaw_offset)) * initial.pose.orientation;
  initial.covariance.diagonal() << 0.5, 0.5, 0.05, 0.001, 0.001, 0.01;
  localizer.initialize(initial, Time::fromSeconds(sequence.times[1]));

  // ---- localize the odd frames ----
  std::mt19937 noise_rng(static_cast<unsigned>(config.seed));
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Pose dead_reckoning = sequence.poses[1];
  Errors filter_errors, dead_reckoning_errors;
  std::vector<double> update_ms, particle_nums;
  size_t refined_count = 0;
  std::ofstream csv;
  if (!config.output.empty()) {
    csv.open(config.output);
    csv << "time,gt_x,gt_y,gt_z,gt_yaw,est_x,est_y,est_z,est_yaw,error,update_ms,particles\n";
  }

  for (size_t k = 3; k < frame_num; k += 2) {
    const double dt = sequence.times[k] - sequence.times[k - 2];
    Twist twist = TwistMotionModel::twistBetween(sequence.poses[k - 2], sequence.poses[k], dt);
    twist.linear *= 1.0 + config.odom_scale_error;
    twist.linear +=
      config.odom_linear_noise * Eigen::Vector3d(standard_normal(noise_rng), 0.0, 0.0);
    twist.angular.z() +=
      config.odom_yaw_rate_bias + config.odom_angular_noise * standard_normal(noise_rng);
    dead_reckoning = TwistMotionModel::integrate(dead_reckoning, twist, dt);
    localizer.addTwist(twist, Time::fromSeconds(sequence.times[k - 2]));

    const PointCloud scan = loadScan(sequence, k);
    const auto start = std::chrono::steady_clock::now();
    const LocalizerStatus status =
      localizer.updateWithPointCloud(scan, Time::fromSeconds(sequence.times[k]));
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (status != LocalizerStatus::kOk) {
      std::cerr << "frame " << k << ": " << toString(status) << "\n";
      continue;
    }

    Pose estimate = localizer.estimate().pose;
    if (localizer.refinedPose()) {
      estimate = *localizer.refinedPose();
    }
    const Pose & truth = sequence.poses[k];
    if ((k - 1) / 2 >= config.warmup_frames) {
      refined_count += localizer.refinedPose() ? 1 : 0;
      filter_errors.add(estimate, truth);
      dead_reckoning_errors.add(dead_reckoning, truth);
      update_ms.push_back(ms);
      particle_nums.push_back(static_cast<double>(localizer.particles().size()));
    }
    if (csv) {
      csv << sequence.times[k] << ',' << truth.position.x() << ',' << truth.position.y() << ','
          << truth.position.z() << ',' << yawOf(truth.orientation) << ',' << estimate.position.x()
          << ',' << estimate.position.y() << ',' << estimate.position.z() << ','
          << yawOf(estimate.orientation) << ','
          << (estimate.position - truth.position).head<2>().norm() << ',' << ms << ','
          << localizer.particles().size() << '\n';
    }
  }

  // ---- report ----
  const Summary t = summarize(filter_errors.translation);
  const Summary y = summarize(filter_errors.yaw);
  const Summary dr = summarize(dead_reckoning_errors.translation);
  const Summary time = summarize(update_ms);
  const Summary particles = summarize(particle_nums);
  const size_t lost = static_cast<size_t>(std::count_if(
    filter_errors.translation.begin(), filter_errors.translation.end(),
    [](double e) { return e > 1.0; }));
  std::printf(
    "map=%s res=%.2f device=%s threads=%zu resampler=%s particles=%zu sample=%zu ndt=%d reset=%d "
    "seed=%lu\n",
    config.map_type.c_str(), config.map_resolution, config.device.c_str(), config.threads,
    config.resampler.c_str(), config.particles, config.sample, config.scan_matching,
    config.expansion_reset, static_cast<unsigned long>(config.seed));
  std::printf(
    "  map: %zu points, built in %.1f s, %s\n", map->size(), map_seconds,
    point_cloud_map->description().c_str());
  std::printf(
    "  frames: %zu evaluated, lost (>1 m): %zu, refined by NDT: %zu\n",
    filter_errors.translation.size(), lost, refined_count);
  std::printf(
    "  translation [m]  rmse %.3f mean %.3f p95 %.3f max %.3f\n", t.rmse, t.mean, t.p95, t.max);
  std::printf(
    "  yaw [deg]        rmse %.3f mean %.3f p95 %.3f max %.3f\n", y.rmse * 180 / M_PI,
    y.mean * 180 / M_PI, y.p95 * 180 / M_PI, y.max * 180 / M_PI);
  std::printf("  dead reckoning   rmse %.3f max %.3f [m]\n", dr.rmse, dr.max);
  std::printf("  update [ms]      mean %.1f p95 %.1f max %.1f\n", time.mean, time.p95, time.max);
  std::printf("  particles        mean %.0f max %.0f\n", particles.mean, particles.max);
  return 0;
}
