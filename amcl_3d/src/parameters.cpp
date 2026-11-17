#include "amcl_3d/parameters.hpp"

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

}  // namespace

NodeParams declareNodeParams(rclcpp::Node & node)
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
  p.publish_tf = declare<bool>(node, "publish_tf", true, "Publish the map->odom transform");
  p.publish_particles =
    declare<bool>(node, "publish_particles", true, "Publish the particles as a PoseArray");
  p.default_initial_pose_stddev = declareVector6(
    node, "default_initial_pose_stddev", (Vector6d() << 0.5, 0.5, 0.1, 0.02, 0.02, 0.2).finished(),
    "Std dev used when the initial pose has no covariance: x, y, z [m], roll, pitch, yaw [rad]");
  p.use_imu = declare<bool>(
    node, "use_imu", false, "Use the IMU angular velocity instead of the odometry one");

  p.map.type = toMapType(declare<std::string>(
    node, "map.type", "distance_field", "'distance_field' (fast) or 'kdtree' (exact, slow)"));
  p.map.device = toMapDevice(declare<std::string>(
    node, "map.device", "cpu", "'cpu' or 'cuda' (distance_field only, needs a CUDA build)"));
  p.map.resolution =
    declare<double>(node, "map.resolution", 0.2, "Voxel size of the distance field [m]");
  validate(p.map);  // fail at startup, not when the map arrives

  p.global_localization.particle_num = declareCount(
    node, "global_localization.particle_num", 20000, "Particles drawn over the whole map");
  p.global_localization.sampler.cell_size = declare<double>(
    node, "global_localization.cell_size", 1.0, "Grid cell size to find the ground [m]");
  p.global_localization.sampler.base_height = declare<double>(
    node, "global_localization.base_height", 0.0, "Height of the base frame above the ground [m]");

  const NdtScanMatcher::Params ndt_defaults;
  auto & matching = p.scan_matching;
  matching.enabled =
    declare<bool>(node, "scan_matching.enabled", false, "Refine the estimate by NDT scan matching");
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

LocalizerParams declareLocalizerParams(rclcpp::Node & node)
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
  const auto seed =
    declare<int64_t>(node, "random_seed", -1, "Seed of the random generator (-1: random)");
  p.random_seed = seed < 0 ? std::random_device()() : static_cast<uint64_t>(seed);
  // One knob for every parallel step of the filter.
  const size_t thread_num = declareCount(
    node, "thread_num", defaults.point_cloud_measurement.thread_num,
    "CPU threads of the filter (0: all hardware threads, often slower)");
  p.point_cloud_measurement.thread_num = thread_num;

  auto & motion = p.motion_model;
  motion.linear_noise_scale = declare<double>(
    node, "motion_model.linear_noise_scale", defaults.motion_model.linear_noise_scale,
    "Linear velocity noise proportional to |v| [1/sqrt(s)]");
  motion.linear_noise_bias = declare<double>(
    node, "motion_model.linear_noise_bias", defaults.motion_model.linear_noise_bias,
    "Constant linear velocity noise [m/s*sqrt(s)]");
  motion.angular_noise_scale = declare<double>(
    node, "motion_model.angular_noise_scale", defaults.motion_model.angular_noise_scale,
    "Angular velocity noise proportional to |w| [1/sqrt(s)]");
  motion.angular_noise_bias = declare<double>(
    node, "motion_model.angular_noise_bias", defaults.motion_model.angular_noise_bias,
    "Constant angular velocity noise [rad/s*sqrt(s)]");

  auto & measurement = p.point_cloud_measurement;
  measurement.max_distance = declare<double>(
    node, "point_cloud_measurement.max_distance", defaults.point_cloud_measurement.max_distance,
    "Point-to-map distances are clipped here [m]");
  measurement.sigma = declare<double>(
    node, "point_cloud_measurement.sigma", defaults.point_cloud_measurement.sigma,
    "Std dev of the point-to-map distance [m]");

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
  expansion.likelihood_threshold = declare<double>(
    node, "expansion_reset.likelihood_threshold", defaults.expansion_reset.likelihood_threshold,
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

}  // namespace amcl_3d
