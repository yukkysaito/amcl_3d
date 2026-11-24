#include "amcl_3d/parameters.hpp"

#include "amcl_3d_core/motion_model/motion_model_factory.hpp"

#include <random>
#include <stdexcept>
#include <vector>

namespace amcl_3d
{
namespace
{
// All parameters are read once at startup, hence read_only.
template <typename T>
T declare(
  rclcpp::Node & node, const std::string & name, const T & default_value,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, default_value, descriptor);
}

size_t declareCount(
  rclcpp::Node & node, const std::string & name, size_t default_value,
  const std::string & description)
{
  const auto value = declare<int64_t>(node, name, static_cast<int64_t>(default_value), description);
  if (value < 0) {
    throw std::invalid_argument("parameter '" + name + "' must not be negative");
  }
  return static_cast<size_t>(value);
}

Vector6d declareVector6(
  rclcpp::Node & node, const std::string & name, const Vector6d & default_value,
  const std::string & description)
{
  const auto value = declare<std::vector<double>>(
    node, name, std::vector<double>(default_value.data(), default_value.data() + 6), description);
  if (value.size() != 6) {
    throw std::invalid_argument("parameter '" + name + "' must have 6 elements");
  }
  return Eigen::Map<const Vector6d>(value.data());
}

ResamplerType toResamplerType(const std::string & name)
{
  if (name == "systematic") {
    return ResamplerType::kSystematic;
  }
  if (name == "kld") {
    return ResamplerType::kKld;
  }
  throw std::invalid_argument("parameter 'resampling.type' must be 'systematic' or 'kld': " + name);
}

EstimateType toEstimateType(const std::string & name)
{
  if (name == "best_cluster") {
    return EstimateType::kBestCluster;
  }
  if (name == "mean") {
    return EstimateType::kMean;
  }
  throw std::invalid_argument(
    "parameter 'estimate.type' must be 'best_cluster' or 'mean': " + name);
}

DelayedFusion toDelayedFusion(const std::string & name)
{
  if (name == "replay") {
    return DelayedFusion::kReplay;
  }
  if (name == "reweight") {
    return DelayedFusion::kReweight;
  }
  throw std::invalid_argument(
    "parameter 'scan_delayed_fusion' must be 'replay' or 'reweight': " + name);
}

NodeParams declareNodeParams(rclcpp::Node & node, size_t thread_num)
{
  NodeParams p;
  p.global_frame_id =
    declare<std::string>(node, "global_frame_id", "map", "Frame of the map and the estimated pose");
  p.odom_frame_id = declare<std::string>(node, "odom_frame_id", "odom", "Odometry frame");
  p.base_frame_id = declare<std::string>(node, "base_frame_id", "base_link", "Vehicle frame");
  p.map_pcd_file = declare<std::string>(
    node, "map_pcd_file", "", "PCD map loaded at startup (empty: subscribe to the map topic)");
  p.transform_tolerance = declare<double>(
    node, "transform_tolerance", 0.1, "Future dating of the map->odom transform [s]");
  p.tf_timeout = declare<double>(node, "tf_timeout", 0.1, "Timeout of transform lookups [s]");
  p.max_stamp_lead = declare<double>(
    node, "max_stamp_lead", 1.0,
    "Inputs stamped more than this ahead of the node clock are dropped [s]");
  if (!(p.max_stamp_lead >= 0.0)) {
    throw std::invalid_argument("parameter 'max_stamp_lead' must not be negative");
  }
  p.time_jump_threshold = declare<double>(
    node, "time_jump_threshold", 5.0,
    "A scan this much older than the newest one restarts the filter at its estimate [s]");
  p.publish_tf = declare<bool>(node, "publish_tf", true, "Publish the map->odom transform");
  p.publish_particles =
    declare<bool>(node, "publish_particles", true, "Publish the particles as a PoseArray");
  p.default_initial_pose_stddev = declareVector6(
    node, "default_initial_pose_stddev", (Vector6d() << 0.5, 0.5, 0.1, 0.02, 0.02, 0.2).finished(),
    "Std dev used when the initial pose has no covariance: x, y, z [m], roll, pitch, yaw [rad]");
  p.use_imu = declare<bool>(
    node, "use_imu", false, "Use the IMU angular velocity instead of the odometry one");

  const PointCloudLikelihoodParams likelihood_defaults;
  auto & likelihood = p.point_cloud_likelihood;
  likelihood.setThreadNum(thread_num);
  likelihood.type = toPointCloudLikelihoodType(declare<std::string>(
    node, "point_cloud_likelihood.type", "likelihood_field",
    "'likelihood_field' (distance to the nearest map point), 'ndt' (voxel normal distributions, "
    "small map) or 'gicp' (ndt with the scan covariances)"));

  auto & field = likelihood.likelihood_field;
  const auto & field_defaults = likelihood_defaults.likelihood_field;
  field.model.max_distance = declare<double>(
    node, "point_cloud_likelihood.likelihood_field.max_distance", field_defaults.model.max_distance,
    "Point-to-map distances are clipped here [m]");
  field.model.sigma = declare<double>(
    node, "point_cloud_likelihood.likelihood_field.sigma", field_defaults.model.sigma,
    "Std dev of the point-to-map distance [m]");
  field.map.type = toMapType(declare<std::string>(
    node, "point_cloud_likelihood.likelihood_field.map.type", "distance_field",
    "'distance_field' (fast) or 'kdtree' (exact, slow)"));
  field.map.device = toMapDevice(declare<std::string>(
    node, "point_cloud_likelihood.likelihood_field.map.device", "cpu",
    "'cpu' or 'cuda' (distance_field only, needs a CUDA build)"));
  field.map.resolution = declare<double>(
    node, "point_cloud_likelihood.likelihood_field.map.resolution", field_defaults.map.resolution,
    "Voxel size of the distance field [m]");

  auto & ndt = likelihood.ndt;
  const auto & ndt_likelihood_defaults = likelihood_defaults.ndt;
  ndt.map.resolution = declare<double>(
    node, "point_cloud_likelihood.ndt.resolution", ndt_likelihood_defaults.map.resolution,
    "Voxel size of the normal distributions [m]");
  ndt.map.min_point_num = declareCount(
    node, "point_cloud_likelihood.ndt.min_point_num", ndt_likelihood_defaults.map.min_point_num,
    "Voxels with fewer map points are dropped");
  ndt.model.sigma = declare<double>(
    node, "point_cloud_likelihood.ndt.sigma", ndt_likelihood_defaults.model.sigma,
    "Measurement noise added to every distribution [m]");
  ndt.model.max_mahalanobis_distance = declare<double>(
    node, "point_cloud_likelihood.ndt.max_mahalanobis_distance",
    ndt_likelihood_defaults.model.max_mahalanobis_distance,
    "Per-point clipping of the Mahalanobis distance [std dev]");
  ndt.map.search_neighbors = declare<bool>(
    node, "point_cloud_likelihood.ndt.search_neighbors",
    ndt_likelihood_defaults.map.search_neighbors,
    "Also match with the 6 face-neighboring voxels (wider basin, more accurate)");

  auto & gicp = likelihood.gicp;
  const auto & gicp_defaults = likelihood_defaults.gicp;
  gicp.map.resolution = declare<double>(
    node, "point_cloud_likelihood.gicp.resolution", gicp_defaults.map.resolution,
    "Voxel size of the map distributions [m]");
  gicp.map.min_point_num = declareCount(
    node, "point_cloud_likelihood.gicp.min_point_num", gicp_defaults.map.min_point_num,
    "Voxels with fewer map points are dropped");
  gicp.map.search_neighbors = declare<bool>(
    node, "point_cloud_likelihood.gicp.search_neighbors", gicp_defaults.map.search_neighbors,
    "Also match with the 6 face-neighboring voxels (wider basin, more accurate)");
  gicp.model.sigma = declare<double>(
    node, "point_cloud_likelihood.gicp.sigma", gicp_defaults.model.sigma,
    "Measurement noise added to every pair of distributions [m]");
  gicp.model.max_mahalanobis_distance = declare<double>(
    node, "point_cloud_likelihood.gicp.max_mahalanobis_distance",
    gicp_defaults.model.max_mahalanobis_distance,
    "Per-point clipping of the Mahalanobis distance [std dev]");
  gicp.model.neighbor_num = declareCount(
    node, "point_cloud_likelihood.gicp.neighbor_num", gicp_defaults.model.neighbor_num,
    "Scan points per scan covariance");
  validate(likelihood);  // fail at startup, not when the map arrives

  p.global_localization.particle_num = declareCount(
    node, "global_localization.particle_num", 20000, "Particles drawn over the whole map");
  p.global_localization.sampler.cell_size = declare<double>(
    node, "global_localization.cell_size", 1.0, "Grid cell size to find the ground [m]");
  p.global_localization.sampler.base_height = declare<double>(
    node, "global_localization.base_height", 0.0, "Height of the base frame above the ground [m]");

  const NdtScanMatcher::Params ndt_defaults;
  auto & matching = p.scan_matching;
  matching.enabled =
    declare<bool>(node, "scan_matching.enabled", false, "Refine the estimate by scan matching");
  matching.type =
    declare<std::string>(node, "scan_matching.type", "gicp", "Scan matcher: 'gicp' or 'ndt'");
  if (matching.type != "gicp" && matching.type != "ndt") {
    throw std::invalid_argument(
      "parameter 'scan_matching.type' must be 'gicp' or 'ndt': " + matching.type);
  }
  const GicpScanMatcher::Params gicp_matcher_defaults;
  matching.gicp.max_correspondence_distance = declare<double>(
    node, "scan_matching.gicp.max_correspondence_distance",
    gicp_matcher_defaults.max_correspondence_distance, "Pairs farther apart are dropped [m]");
  matching.gicp.robust_kernel_width = declare<double>(
    node, "scan_matching.gicp.robust_kernel_width", gicp_matcher_defaults.robust_kernel_width,
    "Cauchy kernel width on the Mahalanobis distance of a pair");
  matching.gicp.thread_num = thread_num;
  matching.ndt.resolution = declare<double>(
    node, "scan_matching.ndt.resolution", ndt_defaults.resolution, "NDT voxel size [m]");
  matching.ndt.step_size = declare<double>(
    node, "scan_matching.ndt.step_size", ndt_defaults.step_size, "Maximum line search step [m]");
  matching.ndt.transformation_epsilon = declare<double>(
    node, "scan_matching.ndt.transformation_epsilon", ndt_defaults.transformation_epsilon,
    "Convergence threshold of the transformation change");
  matching.ndt.max_iterations = static_cast<int>(declareCount(
    node, "scan_matching.ndt.max_iterations", ndt_defaults.max_iterations, "Maximum iterations"));
  matching.ndt.max_fitness = declare<double>(
    node, "scan_matching.ndt.max_fitness", ndt_defaults.max_fitness,
    "Results with a larger fitness score (mean squared distance) are rejected [m^2]");
  return p;
}

LocalizerParams declareLocalizerParams(rclcpp::Node & node, size_t thread_num)
{
  const LocalizerParams defaults;
  LocalizerParams p;

  p.initial_particle_num = declareCount(
    node, "initial_particle_num", defaults.initial_particle_num,
    "Particles drawn at initialization");
  if (p.initial_particle_num == 0) {
    throw std::invalid_argument("parameter 'initial_particle_num' must be positive");
  }
  p.scan_sample_num = declareCount(
    node, "scan_sample_num", defaults.scan_sample_num, "Scan points used per update (0: all)");
  p.point_cloud_sensor.max_effective_observation_num = declareCount(
    node, "scan_max_effective_points", defaults.point_cloud_sensor.max_effective_observation_num,
    "A scan counts as at most this many independent points (tempering; 0: no limit)");
  p.point_cloud_sensor.annealing_layer_num = declareCount(
    node, "scan_annealing_layers", defaults.point_cloud_sensor.annealing_layer_num,
    "Annealing layers run before each scan update (annealed particle filter; 0: off)");
  auto & smc = p.point_cloud_sensor.tempered_smc;
  smc.enabled = declare<bool>(
    node, "scan_tempered_smc.enabled", smc.enabled,
    "Fuse scans by adaptive tempered SMC with MCMC moves (exclusive with scan_annealing_layers)");
  smc.max_stages = declareCount(
    node, "scan_tempered_smc.max_stages", smc.max_stages,
    "Most tempering stages per scan; the last one takes the rest of the likelihood");
  smc.mcmc_steps =
    declareCount(node, "scan_tempered_smc.mcmc_steps", smc.mcmc_steps, "MCMC steps per stage");
  {
    using Kernel = SensorParams::TemperedSmc::Kernel;
    const std::string kernel = declare<std::string>(
      node, "scan_tempered_smc.kernel", "random_walk",
      "MCMC move: random_walk, mala, preconditioned_mala or gauss_newton");
    if (kernel == "random_walk") {
      smc.kernel = Kernel::kRandomWalk;
    } else if (kernel == "mala") {
      smc.kernel = Kernel::kMala;
    } else if (kernel == "preconditioned_mala") {
      smc.kernel = Kernel::kPreconditionedMala;
    } else if (kernel == "gauss_newton") {
      smc.kernel = Kernel::kGaussNewton;
    } else {
      throw std::invalid_argument("parameter 'scan_tempered_smc.kernel' is unknown: " + kernel);
    }
  }
  smc.cess_ratio = declare<double>(
    node, "scan_tempered_smc.cess_ratio", smc.cess_ratio,
    "Conditional ESS / N kept by a tempering stage: lower takes fewer, larger stages");
  if (!(smc.cess_ratio > 0.0 && smc.cess_ratio < 1.0)) {
    throw std::invalid_argument("parameter 'scan_tempered_smc.cess_ratio' must be in (0, 1)");
  }
  smc.thread_num = thread_num;
  if (smc.enabled && p.point_cloud_sensor.annealing_layer_num > 0) {
    throw std::invalid_argument(
      "parameters 'scan_tempered_smc.enabled' and 'scan_annealing_layers' are exclusive");
  }
  if (smc.enabled && smc.max_stages == 0) {
    throw std::invalid_argument("parameter 'scan_tempered_smc.max_stages' must be positive");
  }
  p.max_measurement_delay = declare<double>(
    node, "max_measurement_delay", defaults.max_measurement_delay,
    "Measurements and odometry up to this much older than the newest scan are fused at their "
    "own time by replaying the newer ones [s] (0: older inputs are dropped)");
  if (!(p.max_measurement_delay >= 0.0)) {
    throw std::invalid_argument("parameter 'max_measurement_delay' must not be negative");
  }
  p.point_cloud_sensor.delayed_fusion = toDelayedFusion(declare<std::string>(
    node, "scan_delayed_fusion", "replay",
    "Fusion of a late scan: 'replay' the newer scans (exact) or 'reweight' the descendants of the "
    "particles at its time (one update whatever the delay, approximate)"));
  const auto seed =
    declare<int64_t>(node, "random_seed", -1, "Seed of the random generator (-1: random)");
  p.random_seed = seed < 0 ? std::random_device()() : static_cast<uint64_t>(seed);

  auto & motion = p.motion_model;
  motion.type = toMotionModelType(declare<std::string>(
    node, "motion_model.type", "twist", "'twist' (generic 6-DoF) or 'vehicle' (car-like)"));
  auto & twist = motion.twist;
  const auto & twist_defaults = defaults.motion_model.twist;
  twist.thread_num = thread_num;
  twist.linear_noise_scale = declare<double>(
    node, "motion_model.twist.linear_noise_scale", twist_defaults.linear_noise_scale,
    "Linear velocity noise proportional to |v| [1/sqrt(s)]");
  twist.linear_noise_bias = declare<double>(
    node, "motion_model.twist.linear_noise_bias", twist_defaults.linear_noise_bias,
    "Constant linear velocity noise [m/s*sqrt(s)]");
  twist.angular_noise_scale = declare<double>(
    node, "motion_model.twist.angular_noise_scale", twist_defaults.angular_noise_scale,
    "Angular velocity noise proportional to |w| [1/sqrt(s)]");
  twist.angular_noise_bias = declare<double>(
    node, "motion_model.twist.angular_noise_bias", twist_defaults.angular_noise_bias,
    "Constant angular velocity noise [rad/s*sqrt(s)]");
  auto & vehicle = motion.vehicle;
  const auto & vehicle_defaults = defaults.motion_model.vehicle;
  vehicle.thread_num = thread_num;
  vehicle.speed_noise_scale = declare<double>(
    node, "motion_model.vehicle.speed_noise_scale", vehicle_defaults.speed_noise_scale,
    "Forward speed noise proportional to |v| [1/sqrt(s)]");
  vehicle.speed_noise_bias = declare<double>(
    node, "motion_model.vehicle.speed_noise_bias", vehicle_defaults.speed_noise_bias,
    "Constant forward speed noise while moving [m/s*sqrt(s)]");
  vehicle.yaw_rate_noise_scale = declare<double>(
    node, "motion_model.vehicle.yaw_rate_noise_scale", vehicle_defaults.yaw_rate_noise_scale,
    "Yaw rate noise proportional to |w| [1/sqrt(s)]");
  vehicle.yaw_rate_noise_bias = declare<double>(
    node, "motion_model.vehicle.yaw_rate_noise_bias", vehicle_defaults.yaw_rate_noise_bias,
    "Constant yaw rate noise while moving [rad/s*sqrt(s)]");
  vehicle.lateral_noise_scale = declare<double>(
    node, "motion_model.vehicle.lateral_noise_scale", vehicle_defaults.lateral_noise_scale,
    "Lateral slip velocity noise per |v| [1/sqrt(s)]");
  vehicle.vertical_noise_scale = declare<double>(
    node, "motion_model.vehicle.vertical_noise_scale", vehicle_defaults.vertical_noise_scale,
    "Vertical slip velocity noise per |v| (suspension, base frame misalignment) [1/sqrt(s)]");
  vehicle.attitude_noise_scale = declare<double>(
    node, "motion_model.vehicle.attitude_noise_scale", vehicle_defaults.attitude_noise_scale,
    "Roll / pitch rate noise per |v| (road grade and bank changes) [rad/m*sqrt(s)]");
  vehicle.min_noise_speed = declare<double>(
    node, "motion_model.vehicle.min_noise_speed", vehicle_defaults.min_noise_speed,
    "The noise is computed for at least this speed, so that the estimate refines while stopped "
    "[m/s]");

  auto & resampling = p.resampling;
  resampling.type = toResamplerType(
    declare<std::string>(node, "resampling.type", "systematic", "'systematic' or 'kld'"));
  resampling.ess_ratio_threshold = declare<double>(
    node, "resampling.ess_ratio_threshold", defaults.resampling.ess_ratio_threshold,
    "Resample when ESS / N is below this ratio");
  auto & kld = resampling.kld;
  kld.min_particle_num = declareCount(
    node, "resampling.kld.min_particle_num", defaults.resampling.kld.min_particle_num,
    "Minimum particle count");
  kld.max_particle_num = declareCount(
    node, "resampling.kld.max_particle_num", defaults.resampling.kld.max_particle_num,
    "Maximum particle count");
  kld.epsilon = declare<double>(
    node, "resampling.kld.epsilon", defaults.resampling.kld.epsilon, "Allowed KL divergence");
  kld.delta = declare<double>(
    node, "resampling.kld.delta", defaults.resampling.kld.delta,
    "Probability that the error exceeds epsilon");
  kld.bin_size_xyz = declare<double>(
    node, "resampling.kld.bin_size_xyz", defaults.resampling.kld.bin_size_xyz,
    "Histogram bin size in x, y, z [m]");
  kld.bin_size_yaw = declare<double>(
    node, "resampling.kld.bin_size_yaw", defaults.resampling.kld.bin_size_yaw,
    "Histogram bin size in yaw [rad]");

  auto & estimate = p.estimate;
  estimate.type = toEstimateType(declare<std::string>(
    node, "estimate.type", "best_cluster",
    "'best_cluster' (mean of the most likely cluster of particles) or 'mean' (of all particles)"));
  estimate.clustering.bin_size_xyz = declare<double>(
    node, "estimate.clustering.bin_size_xyz", defaults.estimate.clustering.bin_size_xyz,
    "Clustering grid in x, y, z [m]; particles in touching cells form a cluster");
  estimate.clustering.bin_size_yaw = declare<double>(
    node, "estimate.clustering.bin_size_yaw", defaults.estimate.clustering.bin_size_yaw,
    "Clustering grid in yaw [rad]");

  auto & filter = p.scan_filter;
  filter.min_range = declare<double>(node, "scan_filter.min_range", 1.0, "Minimum point range [m]");
  filter.max_range =
    declare<double>(node, "scan_filter.max_range", 100.0, "Maximum point range [m]");
  filter.min_z = declare<double>(
    node, "scan_filter.min_z", -100.0, "Minimum point height in the base frame [m]");
  filter.max_z =
    declare<double>(node, "scan_filter.max_z", 100.0, "Maximum point height in the base frame [m]");
  filter.voxel_size = declare<double>(
    node, "scan_filter.voxel_size", 0.5, "Downsampling voxel size [m] (0: disabled)");

  auto & expansion = p.expansion_reset;
  expansion.enabled = declare<bool>(
    node, "expansion_reset.enabled", defaults.expansion_reset.enabled,
    "Spread the particles when even the best one explains the scan poorly");
  // the scans are the only sensor of the node that drives recovery
  p.point_cloud_sensor.expansion_reset_threshold = declare<double>(
    node, "expansion_reset.likelihood_threshold",
    defaults.point_cloud_sensor.expansion_reset_threshold,
    "Per-point likelihood of the best particle below which the particles are spread");
  expansion.stddev = declareVector6(
    node, "expansion_reset.stddev", defaults.expansion_reset.stddev,
    "Spread: x, y, z [m], roll, pitch, yaw [rad]");

  auto & matching = p.scan_matching;
  matching.max_correction = declare<double>(
    node, "scan_matching.max_correction", defaults.scan_matching.max_correction,
    "Matched poses farther than this from the estimate are rejected [m]");
  matching.feedback = declare<bool>(
    node, "scan_matching.feedback", defaults.scan_matching.feedback,
    "Also correct the particles with the matched pose");
  matching.feedback_stddev = declareVector6(
    node, "scan_matching.feedback_stddev", defaults.scan_matching.feedback_stddev,
    "Std of the matched pose for the feedback: x, y, z [m], roll, pitch, yaw [rad]");

  auto & augmented = p.augmented_mcl;
  augmented.enabled = declare<bool>(
    node, "augmented_mcl.enabled", defaults.augmented_mcl.enabled,
    "Inject random particles on localization failure");
  augmented.alpha_slow = declare<double>(
    node, "augmented_mcl.alpha_slow", defaults.augmented_mcl.alpha_slow,
    "Decay rate of the long-term likelihood average");
  augmented.alpha_fast = declare<double>(
    node, "augmented_mcl.alpha_fast", defaults.augmented_mcl.alpha_fast,
    "Decay rate of the short-term likelihood average");
  augmented.injection_stddev = declareVector6(
    node, "augmented_mcl.injection_stddev", defaults.augmented_mcl.injection_stddev,
    "Perturbation std of injected particles: x, y, z [m], roll, pitch, yaw [rad]");

  return p;
}

}  // namespace

Amcl3dParams declareParams(rclcpp::Node & node)
{
  // One knob for every parallel step of the filter.
  const size_t thread_num = declareCount(
    node, "thread_num", LikelihoodField::Params{}.thread_num,
    "CPU threads of the filter (0: all hardware threads, often slower)");
  Amcl3dParams params{
    declareLocalizerParams(node, thread_num), declareNodeParams(node, thread_num)};
  if (!(params.node.time_jump_threshold > params.localizer.max_measurement_delay)) {
    throw std::invalid_argument(
      "parameter 'time_jump_threshold' must be larger than 'max_measurement_delay'");
  }
  return params;
}

}  // namespace amcl_3d
