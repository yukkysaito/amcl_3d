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
// Options (defaults in Config), also read from config=<file> with one `key: value` per line
// (command line options take precedence):
// likelihood=likelihood_field|ndt|gicp sigma=<m>
// map=distance_field|kdtree device=cpu|cuda resolution=<m> (likelihood field)
// voxel_resolution=<m> voxel_neighbors=0|1 (NDT, GICP)
// effective=<n> (max effective scan points, 0: no tempering) max_delay=<s>
// late_every=<n> late_by=<scans> (every n-th scan arrives after the next `late_by` scans)
// late_fusion=replay|reweight estimate=cluster|mean
// init_offset=<m> init_yaw_offset=<rad>
// threads=<n> motion=twist|vehicle odometry=full|wheel base_x=<m> base_pitch=<rad>
// resampler=systematic|kld annealing=<layers>
// particles=<n> sample=<n> scan_matching=0|1 scan_matcher=gicp|ndt max_correction=<m>
// expansion_reset=0|1 seed=<n> frames=<n>
// linear_noise_scale= linear_noise_bias= angular_noise_scale= angular_noise_bias= (twist motion model)
// update=importance|annealing|tsmc max_stages=<n> mcmc_steps=<n> kernel=rwm|mala|pmala|gn
// (scan update; tsmc: tempered SMC with that MCMC kernel)
// kidnap_frame=<frame> kidnap_offset=<m> (re-initialize off the truth to measure recovery)
// output=<trajectory.csv> summary=<file> (one JSON line of metrics, appended)
#include <algorithm>
#include <array>
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

#include <pcl/common/transforms.h>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood_factory.hpp"
#include "amcl_3d_core/scan_filter.hpp"
#include "amcl_3d_core/scan_matcher/gicp_scan_matcher.hpp"
#include "amcl_3d_core/scan_matcher/ndt_scan_matcher.hpp"
#include "amcl_3d_core/update/tempered_smc_update.hpp"
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
  std::string likelihood = "likelihood_field";
  double sigma = 0.2;             // [m] of the selected likelihood
  double voxel_resolution = 1.0;  // NDT, GICP
  bool voxel_neighbors = true;
  std::string map_type = "distance_field";
  std::string device = "cpu";  // cuda: likelihood of the distance field on the GPU
  double map_resolution = 0.2;
  size_t threads = 4;            // likelihood threads, 0: all
  std::string motion = "twist";  // motion model: twist | vehicle
  /// Odometry fed to the filter: "full" (all 6 twist components of the ground truth, corrupted) or
  /// "wheel" (forward speed and yaw rate only, like wheel odometry with a gyro).
  std::string odometry = "full";
  /// x of the velodyne in the localized base frame [m]. 0.86 puts the base at the rear axle center
  /// (estimated from the ground truth as the point with the least lateral velocity), where the
  /// non-holonomic vehicle model holds.
  double base_x = 0.0;
  /// Pitch [rad] of the velodyne in the base frame. 0.0143 aligns the base x axis with the
  /// direction of travel (estimated from the ground truth, mean vz / vx = 0.0143).
  double base_pitch = 0.0;
  std::string resampler = "systematic";
  size_t particles = 500;
  size_t sample = 200;
  bool scan_matching = false;
  size_t annealing = 0;  // point_cloud_sensor.annealing_layer_num
  std::string update;    // importance | annealing (annealing layers, 5 if 0) | tsmc
  size_t max_stages = TemperedSmcUpdate::Params{}.max_stages;
  size_t mcmc_steps = TemperedSmcUpdate::Params{}.mcmc_steps;
  std::string kernel = "rwm";  // rwm | mala | pmala | gn
  double cess = 0.5;
  std::string summary;
  size_t kidnap_frame = 0;       // 0: none
  double kidnap_offset = 5.0;    // [m] along x
  double converged_error = 0.5;  // [m]
  std::string scan_matcher = "gicp";
  double max_correction = LocalizerParams{}.scan_matching.max_correction;  // [m]
  TwistMotionModel::Params twist_noise;
  bool expansion_reset = false;
  uint64_t seed = 1;
  size_t frames = 0;       // 0: all
  size_t effective = 10;   // point_cloud_sensor.max_effective_observation_num
  double max_delay = 1.0;  // [s]
  size_t late_every = 0;   // 0: every scan in time order
  size_t late_by = 2;
  std::string late_fusion = "replay";
  std::string estimate = "cluster";
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
  // options from a file of `key: value` lines (# comments); the command line takes precedence
  if (const auto it = kv.find("config"); it != kv.end()) {
    std::ifstream file(it->second);
    if (!file) {
      throw std::invalid_argument("cannot read config: " + it->second);
    }
    kv.erase(it);
    std::string line;
    while (std::getline(file, line)) {
      line = line.substr(0, line.find('#'));
      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      std::istringstream key_stream(line.substr(0, colon)), value_stream(line.substr(colon + 1));
      std::string key, value;
      key_stream >> key;
      value_stream >> value;
      if (!key.empty() && !value.empty()) {
        kv.emplace(key, value);  // does not replace a command line option
      }
    }
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
  take("likelihood", c.likelihood);
  take("sigma", c.sigma);
  take("voxel_resolution", c.voxel_resolution);
  take("voxel_neighbors", c.voxel_neighbors);
  take("map", c.map_type);
  take("resolution", c.map_resolution);
  take("threads", c.threads);
  take("device", c.device);
  take("motion", c.motion);
  take("odometry", c.odometry);
  take("base_x", c.base_x);
  take("base_pitch", c.base_pitch);
  take("resampler", c.resampler);
  take("particles", c.particles);
  take("sample", c.sample);
  take("scan_matching", c.scan_matching);
  take("annealing", c.annealing);
  take("update", c.update);
  take("max_stages", c.max_stages);
  take("mcmc_steps", c.mcmc_steps);
  take("kernel", c.kernel);
  take("cess", c.cess);
  take("summary", c.summary);
  take("kidnap_frame", c.kidnap_frame);
  take("kidnap_offset", c.kidnap_offset);
  take("converged_error", c.converged_error);
  take("scan_matcher", c.scan_matcher);
  take("max_correction", c.max_correction);
  take("linear_noise_scale", c.twist_noise.linear_noise_scale);
  take("linear_noise_bias", c.twist_noise.linear_noise_bias);
  take("angular_noise_scale", c.twist_noise.angular_noise_scale);
  take("angular_noise_bias", c.twist_noise.angular_noise_bias);
  take("expansion_reset", c.expansion_reset);
  take("seed", c.seed);
  take("frames", c.frames);
  take("effective", c.effective);
  take("max_delay", c.max_delay);
  take("late_every", c.late_every);
  take("late_by", c.late_by);
  take("late_fusion", c.late_fusion);
  take("estimate", c.estimate);
  take("init_offset", c.init_offset);
  take("init_yaw_offset", c.init_yaw_offset);
  take("output", c.output);
  if (!kv.empty()) {
    throw std::invalid_argument("unknown option: " + kv.begin()->first);
  }
  if (c.odometry != "full" && c.odometry != "wheel") {
    throw std::invalid_argument("odometry must be full or wheel: " + c.odometry);
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
  params.point_cloud_sensor.max_effective_observation_num = config.effective;
  if (config.update == "tsmc") {
    auto & smc = params.point_cloud_sensor.tempered_smc;
    smc.enabled = true;
    smc.max_stages = config.max_stages;
    smc.mcmc_steps = config.mcmc_steps;
    smc.cess_ratio = config.cess;
    using Kernel = SensorParams::TemperedSmc::Kernel;
    const std::map<std::string, Kernel> kernels{
      {"rwm", Kernel::kRandomWalk},
      {"mala", Kernel::kMala},
      {"pmala", Kernel::kPreconditionedMala},
      {"gn", Kernel::kGaussNewton}};
    if (kernels.count(config.kernel) == 0) {
      throw std::invalid_argument("unknown kernel: " + config.kernel);
    }
    smc.kernel = kernels.at(config.kernel);
    smc.thread_num = config.threads;
  } else if (config.update == "annealing" || (config.update.empty() && config.annealing > 0)) {
    params.point_cloud_sensor.annealing_layer_num = config.annealing > 0 ? config.annealing : 5;
  } else if (!config.update.empty() && config.update != "importance") {
    throw std::invalid_argument("unknown update: " + config.update);
  }
  params.max_measurement_delay = config.max_delay;
  if (config.late_fusion == "reweight") {
    params.point_cloud_sensor.delayed_fusion = DelayedFusion::kReweight;
  } else if (config.late_fusion != "replay") {
    throw std::invalid_argument("unknown late_fusion: " + config.late_fusion);
  }
  if (config.estimate == "mean") {
    params.estimate.type = EstimateType::kMean;
  } else if (config.estimate != "cluster") {
    throw std::invalid_argument("unknown estimate: " + config.estimate);
  }
  params.motion_model.type = toMotionModelType(config.motion);
  params.motion_model.twist = config.twist_noise;
  params.motion_model.twist.thread_num = config.threads;
  params.scan_matching.max_correction = config.max_correction;
  params.motion_model.vehicle.thread_num = config.threads;
  Localizer localizer(params);
  PointCloudLikelihoodParams likelihood_params;
  likelihood_params.type = toPointCloudLikelihoodType(config.likelihood);
  likelihood_params.setThreadNum(config.threads);
  auto & field = likelihood_params.likelihood_field;
  field.map.type = toMapType(config.map_type);
  field.map.device = toMapDevice(config.device);
  field.map.resolution = config.map_resolution;
  field.model.sigma = config.sigma;
  auto & ndt = likelihood_params.ndt;
  ndt.map.resolution = config.voxel_resolution;
  ndt.map.search_neighbors = config.voxel_neighbors;
  ndt.model.sigma = config.sigma;
  auto & gicp = likelihood_params.gicp;
  gicp.map = ndt.map;
  gicp.model.sigma = config.sigma;
  const auto likelihood = createPointCloudLikelihood(map, likelihood_params);
  localizer.setPointCloudLikelihood(likelihood);
  if (config.scan_matching) {
    if (config.scan_matcher == "gicp") {
      GicpScanMatcher::Params matcher_params;
      matcher_params.thread_num = config.threads;
      localizer.setScanMatcher(std::make_unique<GicpScanMatcher>(map, matcher_params));
    } else if (config.scan_matcher == "ndt") {
      localizer.setScanMatcher(std::make_unique<NdtScanMatcher>(map, NdtScanMatcher::Params{}));
    } else {
      throw std::invalid_argument("unknown scan_matcher: " + config.scan_matcher);
    }
  }
  const double map_seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - map_start).count();

  // ground truth of the base frame: velodyne pose * velodyne_from_base
  const Eigen::Isometry3d base_from_velodyne =
    Eigen::Translation3d(config.base_x, 0.0, 0.0) *
    Eigen::AngleAxisd(config.base_pitch, Eigen::Vector3d::UnitY());
  std::vector<Pose> truth(sequence.poses.size());
  for (size_t k = 0; k < truth.size(); ++k) {
    const Eigen::Isometry3d world_from_base = Eigen::Translation3d(sequence.poses[k].position) *
                                              sequence.poses[k].orientation *
                                              base_from_velodyne.inverse();
    truth[k].position = world_from_base.translation();
    truth[k].orientation = Eigen::Quaterniond(world_from_base.rotation()).normalized();
  }
  const Eigen::Matrix4f base_from_velodyne_matrix = base_from_velodyne.matrix().cast<float>();

  PoseWithCovariance initial;
  initial.pose = truth[1];
  initial.pose.position.x() += config.init_offset;
  initial.pose.orientation =
    expMap(Eigen::Vector3d(0, 0, config.init_yaw_offset)) * initial.pose.orientation;
  initial.covariance.diagonal() << 0.5, 0.5, 0.05, 0.001, 0.001, 0.01;
  localizer.initialize(initial, Time::fromSeconds(sequence.times[1]));

  // ---- localize the odd frames ----
  std::mt19937 noise_rng(static_cast<unsigned>(config.seed));
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Pose dead_reckoning = truth[1];
  Errors filter_errors, dead_reckoning_errors;
  std::vector<double> update_ms, particle_nums;
  size_t refined_count = 0;
  std::ofstream csv;
  if (!config.output.empty()) {
    csv.open(config.output);
    csv << "time,gt_x,gt_y,gt_z,gt_yaw,est_x,est_y,est_z,est_yaw,error,update_ms,particles\n";
  }

  // Output for each frame: the estimate when that frame was the newest measurement, last delivery
  // wins (a delayed scan replays the newer ones and updates their estimates).
  struct Output
  {
    Pose estimate;
    Pose dead_reckoning;
    bool refined;
    double ms;
    size_t particle_num;
    PoseWithCovariance filter_estimate;  // of the particles (NEES)
    UpdateStatistics statistics;
    double unique_ratio;  // distinct particles / N after the update (diversity)
  };
  std::map<size_t, Output> outputs;
  std::map<int64_t, size_t> frame_of_stamp;
  std::vector<Pose> dead_reckoning_at(frame_num);
  auto deliver = [&](size_t k, const PointCloud & scan) {
    const auto start = std::chrono::steady_clock::now();
    const LocalizerStatus status =
      localizer.updateWithPointCloud(scan, Time::fromSeconds(sequence.times[k]));
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (status != LocalizerStatus::kOk) {
      std::cerr << "frame " << k << ": " << toString(status) << "\n";
      return;
    }
    const size_t newest = frame_of_stamp.at(localizer.stamp().nanoseconds);
    const auto refined = localizer.refinedPose();
    const ParticleSet & particles = localizer.particles();
    std::vector<std::array<double, 3>> positions(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
      const auto row = particles.positions().row(static_cast<Eigen::Index>(i));
      positions[i] = {row(0), row(1), row(2)};
    }
    std::sort(positions.begin(), positions.end());
    const size_t unique =
      static_cast<size_t>(std::unique(positions.begin(), positions.end()) - positions.begin());
    const PoseWithCovariance filter_estimate = localizer.estimate();
    outputs[newest] = Output{
      refined ? *refined : filter_estimate.pose,
      dead_reckoning_at[newest],
      refined.has_value(),
      ms,
      particles.size(),
      filter_estimate,
      localizer.lastUpdateStatistics(),
      static_cast<double>(unique) / static_cast<double>(std::max<size_t>(particles.size(), 1))};
    if ((k - 1) / 2 >= config.warmup_frames) {
      update_ms.push_back(ms);
    }
  };

  struct Late
  {
    size_t frame;
    PointCloud scan;
    size_t release;  // delivered once this many scans have been delivered
  };
  std::vector<Late> late;
  size_t delivered = 0;
  for (size_t k = 3; k < frame_num; k += 2) {
    const double dt = sequence.times[k] - sequence.times[k - 2];
    Twist twist = TwistMotionModel::twistBetween(truth[k - 2], truth[k], dt);
    if (config.odometry == "wheel") {
      twist.linear.y() = twist.linear.z() = 0.0;
      twist.angular.x() = twist.angular.y() = 0.0;
    }
    twist.linear *= 1.0 + config.odom_scale_error;
    twist.linear +=
      config.odom_linear_noise * Eigen::Vector3d(standard_normal(noise_rng), 0.0, 0.0);
    twist.angular.z() +=
      config.odom_yaw_rate_bias + config.odom_angular_noise * standard_normal(noise_rng);
    dead_reckoning = TwistMotionModel::integrate(dead_reckoning, twist, dt);
    dead_reckoning_at[k] = dead_reckoning;
    frame_of_stamp[Time::fromSeconds(sequence.times[k]).nanoseconds] = k;
    if (config.kidnap_frame > 0 && k == (config.kidnap_frame | 1)) {
      // a false relocalization: confident, kidnap_offset off the truth
      PoseWithCovariance wrong;
      wrong.pose = truth[k - 2];
      wrong.pose.position.x() += config.kidnap_offset;
      wrong.covariance.diagonal() << 0.05, 0.05, 0.01, 0.0005, 0.0005, 0.002;
      localizer.initialize(wrong, Time::fromSeconds(sequence.times[k - 2]));
    }
    localizer.addTwist(twist, Time::fromSeconds(sequence.times[k - 2]));

    PointCloud scan;
    pcl::transformPointCloud(loadScan(sequence, k), scan, base_from_velodyne_matrix);
    if (config.late_every > 0 && ((k - 3) / 2) % config.late_every == config.late_every - 1) {
      late.push_back(Late{k, std::move(scan), delivered + config.late_by});
    } else {
      deliver(k, scan);
      ++delivered;
    }
    for (auto it = late.begin(); it != late.end();) {
      if (it->release <= delivered) {
        deliver(it->frame, it->scan);
        it = late.erase(it);
      } else {
        ++it;
      }
    }
  }
  for (const auto & pending : late) {
    deliver(pending.frame, pending.scan);
  }

  for (const auto & [k, output] : outputs) {
    const Pose & true_pose = truth[k];
    if ((k - 1) / 2 >= config.warmup_frames) {
      refined_count += output.refined ? 1 : 0;
      filter_errors.add(output.estimate, true_pose);
      dead_reckoning_errors.add(output.dead_reckoning, true_pose);
      particle_nums.push_back(static_cast<double>(output.particle_num));
    }
    if (csv) {
      const Pose & estimate = output.estimate;
      csv << sequence.times[k] << ',' << true_pose.position.x() << ',' << true_pose.position.y()
          << ',' << true_pose.position.z() << ',' << yawOf(true_pose.orientation) << ','
          << estimate.position.x() << ',' << estimate.position.y() << ',' << estimate.position.z()
          << ',' << yawOf(estimate.orientation) << ','
          << (estimate.position - true_pose.position).head<2>().norm() << ',' << output.ms << ','
          << output.particle_num << '\n';
    }
  }

  // ---- consistency, convergence, diagnostics ----
  std::vector<double> nees, stages, evaluations, acceptance, min_ess, unique_ratios;
  size_t covered = 0;
  std::vector<std::pair<size_t, double>> errors_in_order;  // frame, error (all frames)
  for (const auto & [k, output] : outputs) {
    errors_in_order.emplace_back(
      k, (output.estimate.position - truth[k].position).head<2>().norm());
    if ((k - 1) / 2 < config.warmup_frames) {
      continue;
    }
    // NEES of (x, y, yaw) of the particle estimate with its covariance
    const Pose & e = output.filter_estimate.pose;
    Eigen::Vector3d error;
    error.head<2>() = (e.position - truth[k].position).head<2>();
    const double dyaw = yawOf(e.orientation) - yawOf(truth[k].orientation);
    error(2) = std::atan2(std::sin(dyaw), std::cos(dyaw));
    const Matrix6d & c = output.filter_estimate.covariance;
    Eigen::Matrix3d p;
    p << c(0, 0), c(0, 1), c(0, 5), c(1, 0), c(1, 1), c(1, 5), c(5, 0), c(5, 1), c(5, 5);
    p += 1e-12 * Eigen::Matrix3d::Identity();
    nees.push_back(error.dot(p.ldlt().solve(error)));
    covered += nees.back() <= 7.815 ? 1 : 0;  // chi-square 95 %, 3 dof
    stages.push_back(static_cast<double>(output.statistics.stages));
    evaluations.push_back(static_cast<double>(output.statistics.likelihood_evaluations));
    if (output.statistics.proposals > 0) {
      acceptance.push_back(output.statistics.acceptanceRate());
    }
    min_ess.push_back(output.statistics.min_ess_ratio);
    unique_ratios.push_back(output.unique_ratio);
  }
  // converged from the first frame from which the error stays below converged_error (up to the
  // kidnapping or the end); recovered likewise after the kidnapping
  auto settle = [&](size_t from_frame, size_t to_frame) -> double {
    double settled = -1.0;
    for (const auto & [k, err] : errors_in_order) {
      if (k < from_frame || k >= to_frame) {
        continue;
      }
      if (err >= config.converged_error) {
        settled = -1.0;
      } else if (settled < 0.0) {
        settled = sequence.times[k] - sequence.times[from_frame];
      }
    }
    return settled;
  };
  const size_t kidnap = config.kidnap_frame > 0 ? (config.kidnap_frame | 1) : frame_num;
  const double convergence_time = settle(1, kidnap);
  const double recovery_time = config.kidnap_frame > 0 ? settle(kidnap, frame_num) : -1.0;
  double final_error = 0.0;  // mean of the last 10 frames: far off = converged to a wrong mode
  {
    const size_t count = std::min<size_t>(10, errors_in_order.size());
    for (size_t i = errors_in_order.size() - count; i < errors_in_order.size(); ++i) {
      final_error += errors_in_order[i].second / static_cast<double>(count);
    }
  }
  auto mean = [](const std::vector<double> & v) {
    double sum = 0.0;
    for (const double x : v) {
      sum += x;
    }
    return v.empty() ? 0.0 : sum / static_cast<double>(v.size());
  };

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
    "map=%s res=%.2f device=%s threads=%zu motion=%s odometry=%s base_x=%.2f base_pitch=%.4f "
    "resampler=%s "
    "particles=%zu sample=%zu effective=%zu matching=%s reset=%d late_every=%zu late_by=%zu "
    "late_fusion=%s estimate=%s seed=%lu\n",
    config.map_type.c_str(), config.map_resolution, config.device.c_str(), config.threads,
    config.motion.c_str(), config.odometry.c_str(), config.base_x, config.base_pitch,
    config.resampler.c_str(), config.particles, config.sample, config.effective,
    config.scan_matching ? config.scan_matcher.c_str() : "off", config.expansion_reset,
    config.late_every, config.late_by, config.late_fusion.c_str(), config.estimate.c_str(),
    static_cast<unsigned long>(config.seed));
  std::printf(
    "  map: %zu points, built in %.1f s, %s\n", map->size(), map_seconds,
    likelihood->description().c_str());
  std::printf(
    "  frames: %zu evaluated, lost (>1 m): %zu, refined by scan matching: %zu, replayed updates: "
    "%zu, "
    "reweighted scans: %zu\n",
    filter_errors.translation.size(), lost, refined_count, localizer.replayedCount(),
    localizer.reweightedCount());
  const Pose final_pose = localizer.estimate().pose;  // after every scan arrived
  std::printf(
    "  final estimate: %.9f %.9f %.9f (stamp %.3f)\n", final_pose.position.x(),
    final_pose.position.y(), final_pose.position.z(), localizer.stamp().seconds());
  std::printf(
    "  translation [m]  rmse %.3f mean %.3f p95 %.3f max %.3f\n", t.rmse, t.mean, t.p95, t.max);
  std::printf(
    "  yaw [deg]        rmse %.3f mean %.3f p95 %.3f max %.3f\n", y.rmse * 180 / M_PI,
    y.mean * 180 / M_PI, y.p95 * 180 / M_PI, y.max * 180 / M_PI);
  std::printf("  dead reckoning   rmse %.3f max %.3f [m]\n", dr.rmse, dr.max);
  std::printf("  update [ms]      mean %.1f p95 %.1f max %.1f\n", time.mean, time.p95, time.max);
  std::printf("  particles        mean %.0f max %.0f\n", particles.mean, particles.max);
  const Summary rotation = summarize(filter_errors.rotation);
  std::printf(
    "  rotation [deg]   rmse %.3f p95 %.3f max %.3f (3D)\n", rotation.rmse * 180 / M_PI,
    rotation.p95 * 180 / M_PI, rotation.max * 180 / M_PI);
  std::printf(
    "  NEES (x, y, yaw) mean %.2f (3 if consistent), 95 %% coverage %.1f %%\n", mean(nees),
    nees.empty() ? 0.0 : 100.0 * static_cast<double>(covered) / static_cast<double>(nees.size()));
  std::printf(
    "  update           stages %.2f, likelihood evaluations %.2f, acceptance %.2f, min ESS / N "
    "%.2f, distinct particles %.2f, p99 %.1f ms\n",
    mean(stages), mean(evaluations), mean(acceptance), mean(min_ess), mean(unique_ratios),
    time.p99);
  std::printf(
    "  converged after  %.1f s%s\n", convergence_time,
    config.kidnap_frame > 0 ? (", recovered after " + std::to_string(recovery_time) + " s").c_str()
                            : "");
  if (!config.summary.empty()) {
    std::ofstream out(config.summary, std::ios::app);
    out << "{\"sequence\": \"" << argv[2] << "\", \"update\": \""
        << (config.update.empty() ? (config.annealing > 0 ? "annealing" : "importance")
                                  : config.update)
        << "\", \"seed\": " << config.seed << ", \"particles\": " << config.particles
        << ", \"init_offset\": " << config.init_offset
        << ", \"init_yaw_offset\": " << config.init_yaw_offset
        << ", \"annealing\": " << config.annealing << ", \"max_stages\": " << config.max_stages
        << ", \"mcmc_steps\": " << config.mcmc_steps << ", \"kernel\": \"" << config.kernel
        << "\", \"cess\": " << config.cess << ", \"effective\": " << config.effective
        << ", \"sample\": " << config.sample << ", \"sigma\": " << config.sigma
        << ", \"scan_matching\": " << config.scan_matching << ", \"device\": \"" << config.device
        << "\", \"rmse\": " << t.rmse << ", \"mean\": " << t.mean << ", \"p95\": " << t.p95
        << ", \"max\": " << t.max << ", \"yaw_rmse_deg\": " << y.rmse * 180 / M_PI
        << ", \"rotation_rmse_deg\": " << rotation.rmse * 180 / M_PI << ", \"lost\": " << lost
        << ", \"frames\": " << filter_errors.translation.size() << ", \"nees\": " << mean(nees)
        << ", \"coverage95\": "
        << (nees.empty() ? 0.0 : static_cast<double>(covered) / static_cast<double>(nees.size()))
        << ", \"stages\": " << mean(stages) << ", \"evaluations\": " << mean(evaluations)
        << ", \"acceptance\": " << mean(acceptance) << ", \"min_ess\": " << mean(min_ess)
        << ", \"distinct\": " << mean(unique_ratios) << ", \"ms_mean\": " << time.mean
        << ", \"ms_p95\": " << time.p95 << ", \"ms_p99\": " << time.p99
        << ", \"convergence_s\": " << convergence_time << ", \"recovery_s\": " << recovery_time
        << ", \"kidnap_frame\": " << config.kidnap_frame << ", \"final_error\": " << final_error
        << "}\n";
  }
  return 0;
}
