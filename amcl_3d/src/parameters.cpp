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
