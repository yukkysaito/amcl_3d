#include "amcl_3d/amcl_3d_node.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <stdexcept>
#include <vector>

#include "amcl_3d/conversions.hpp"
#include "amcl_3d_core/scan_matcher/ndt_scan_matcher.hpp"

namespace amcl_3d
{
namespace
{
constexpr int kThrottleMs = 5000;

PointCloud::Ptr toPointCloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(msg, *cloud);
  std::vector<int> unused_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, unused_indices);
  return cloud;
}
}  // namespace

Amcl3dNode::Amcl3dNode(const rclcpp::NodeOptions & options)
: Node("amcl_3d", options),
  params_(declareParams(*this)),
  localizer_(std::make_unique<Localizer>(params_.localizer)),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(*this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", rclcpp::QoS(10));
  particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particles", rclcpp::QoS(1));

  if (params_.node.map_pcd_file.empty()) {
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(1).transient_local().reliable(), std::bind(&Amcl3dNode::onMap, this, _1));
  } else {
    PointCloud::Ptr map(new PointCloud);
    if (pcl::io::loadPCDFile(params_.node.map_pcd_file, *map) != 0) {
      throw std::runtime_error("failed to load the map: " + params_.node.map_pcd_file);
    }
    setMap(map);
  }
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1), std::bind(&Amcl3dNode::onInitialPose, this, _1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS().keep_last(100), std::bind(&Amcl3dNode::onOdometry, this, _1));
  if (params_.node.use_imu) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS().keep_last(100), std::bind(&Amcl3dNode::onImu, this, _1));
  }
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points", rclcpp::SensorDataQoS().keep_last(1), std::bind(&Amcl3dNode::onPointCloud, this, _1));
  global_localization_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/global_localization", std::bind(&Amcl3dNode::onGlobalLocalization, this, _1, _2));
}

std::optional<Eigen::Isometry3d> Amcl3dNode::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & stamp)
{
  if (target_frame == source_frame) {
    return Eigen::Isometry3d::Identity();
  }
  try {
    const auto transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(params_.node.tf_timeout));
    return tf2::transformToEigen(transform);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "%s", e.what());
    return std::nullopt;
  }
}

void Amcl3dNode::setMap(const PointCloud::Ptr & map)
{
  try {
    const auto likelihood = createPointCloudLikelihood(map, params_.node.point_cloud_likelihood);
    RCLCPP_INFO(get_logger(), "%s", likelihood->description().c_str());
    localizer_->setPointCloudLikelihood(likelihood);
    global_sampler_ =
      std::make_unique<GroundPoseSampler>(*map, params_.node.global_localization.sampler);
    if (params_.node.scan_matching.enabled) {
      localizer_->setScanMatcher(
        std::make_unique<NdtScanMatcher>(map, params_.node.scan_matching.ndt));
    }
    RCLCPP_INFO(get_logger(), "map received: %zu points", map->size());
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_logger(), "invalid map: %s", e.what());
  }
}

void Amcl3dNode::onMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const auto global_from_map =
    lookupTransform(params_.node.global_frame_id, msg->header.frame_id, rclcpp::Time(0));
  if (!global_from_map) {
    RCLCPP_ERROR(get_logger(), "cannot transform the map from %s", msg->header.frame_id.c_str());
    return;
  }
  PointCloud::Ptr map = toPointCloud(*msg);
  pcl::transformPointCloud(*map, *map, global_from_map->matrix().cast<float>());
  setMap(map);
}

void Amcl3dNode::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  const auto global_from_msg =
    lookupTransform(params_.node.global_frame_id, msg->header.frame_id, rclcpp::Time(0));
  if (!global_from_msg) {
    RCLCPP_ERROR(
      get_logger(), "cannot transform the initial pose from %s", msg->header.frame_id.c_str());
    return;
  }
  PoseWithCovariance initial_pose = transform(fromMsg(msg->pose), *global_from_msg);
  if (initial_pose.covariance.diagonal().isZero()) {
    RCLCPP_WARN(get_logger(), "initial pose has no covariance, using default_initial_pose_stddev");
    initial_pose.covariance = params_.node.default_initial_pose_stddev.cwiseAbs2().asDiagonal();
  }
  // RViz stamps with its own clock, which may not be the node's (use_sim_time)
  rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (stamp.nanoseconds() == 0 || isAhead(stamp)) {
    stamp = now();
  }
  try {
    localizer_->initialize(initial_pose, Time{stamp.nanoseconds()});
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_logger(), "initial pose rejected: %s", e.what());
    return;
  }
  reported_expansion_resets_ = 0;

  const Pose & pose = initial_pose.pose;
  RCLCPP_INFO(
    get_logger(), "initialized at (%.2f, %.2f, %.2f) with %zu particles", pose.position.x(),
    pose.position.y(), pose.position.z(), localizer_->particles().size());
  publishEstimate(stamp);
}

void Amcl3dNode::onGlobalLocalization(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (!global_sampler_) {
    response->success = false;
    response->message = "no map";
    return;
  }
  const rclcpp::Time stamp = now();
  localizer_->initialize(
    [this](RandomGenerator & rng) { return global_sampler_->sample(rng); },
    params_.node.global_localization.particle_num, Time{stamp.nanoseconds()});
  reported_expansion_resets_ = 0;
  if (params_.localizer.resampling.type != ResamplerType::kKld) {
    RCLCPP_WARN(
      get_logger(),
      "global localization with systematic resampling keeps all %zu particles; use kld",
      params_.node.global_localization.particle_num);
  }
  response->success = true;
  response->message = "spread " + std::to_string(localizer_->particles().size()) +
                      " particles over " + std::to_string(global_sampler_->cellNum()) + " cells";
  RCLCPP_INFO(get_logger(), "global localization: %s", response->message.c_str());
  publishEstimate(stamp);
}

bool Amcl3dNode::isAhead(const rclcpp::Time & stamp)
{
  const rclcpp::Time clock = now();
  if ((stamp - clock).seconds() <= params_.node.max_stamp_lead) {
    return false;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), kThrottleMs,
    "input stamped %.3f s ahead of the clock (max_stamp_lead %.3f s): another time source?",
    (stamp - clock).seconds(), params_.node.max_stamp_lead);
  return true;
}

void Amcl3dNode::addTwist(const Twist & twist, const builtin_interfaces::msg::Time & stamp)
{
  if (isAhead(rclcpp::Time(stamp, get_clock()->get_clock_type()))) {
    return;
  }
  const LocalizerStatus status = localizer_->addTwist(twist, fromMsg(stamp));
  if (status != LocalizerStatus::kOk) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs, "twist skipped: %s", toString(status));
  }
}

void Amcl3dNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // nav_msgs/Odometry twist is expressed in child_frame_id, which should be the base frame.
  if (!msg->child_frame_id.empty() && msg->child_frame_id != params_.node.base_frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "odometry child_frame_id '%s' differs from base_frame_id '%s'", msg->child_frame_id.c_str(),
      params_.node.base_frame_id.c_str());
  }
  const Twist twist = fromMsg(msg->twist.twist);
  if (params_.node.use_imu) {
    odom_linear_velocity_ = twist.linear;  // combined with the angular velocity in onImu()
    return;
  }
  addTwist(twist, msg->header.stamp);
}

void Amcl3dNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  if (!odom_linear_velocity_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "waiting for odometry");
    return;
  }
  if (!base_from_imu_) {
    const auto transform =
      lookupTransform(params_.node.base_frame_id, msg->header.frame_id, rclcpp::Time(0));
    if (!transform) {
      return;
    }
    base_from_imu_ = Eigen::Quaterniond(transform->rotation());
  }
  Twist twist;
  twist.linear = *odom_linear_velocity_;
  twist.angular =
    *base_from_imu_ *
    Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  addTwist(twist, msg->header.stamp);
}

void Amcl3dNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (!localizer_->isInitialized()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "waiting for the initial pose");
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (isAhead(stamp)) {
    return;
  }
  if (localizer_->stamp().seconds() - stamp.seconds() > params_.node.time_jump_threshold) {
    // Time jumped back (looping rosbag, restarted simulation): every scan would be stale.
    const PoseWithCovariance estimate = localizer_->estimate();
    RCLCPP_WARN(
      get_logger(), "time jumped back by %.1f s, restarting at the estimate",
      localizer_->stamp().seconds() - stamp.seconds());
    localizer_->initialize(estimate, fromMsg(msg->header.stamp));
  }
  const auto base_from_sensor =
    lookupTransform(params_.node.base_frame_id, msg->header.frame_id, stamp);
  if (!base_from_sensor) {
    return;
  }
  PointCloud::Ptr scan = toPointCloud(*msg);
  pcl::transformPointCloud(*scan, *scan, base_from_sensor->matrix().cast<float>());

  const LocalizerStatus status =
    localizer_->updateWithPointCloud(*scan, fromMsg(msg->header.stamp));
  if (status != LocalizerStatus::kOk) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs, "scan skipped: %s", toString(status));
    return;
  }
  if (localizer_->expansionResetCount() > reported_expansion_resets_) {
    reported_expansion_resets_ = localizer_->expansionResetCount();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "expansion reset (best per-point likelihood %.3f), %zu so far",
      localizer_->lastLikelihood().max, reported_expansion_resets_);
  }
  // A delayed scan updates the past and replays the newer measurements: the estimate is the one at
  // the newest measurement.
  publishEstimate(rclcpp::Time(localizer_->stamp().nanoseconds, get_clock()->get_clock_type()));
}

void Amcl3dNode::publishEstimate(const rclcpp::Time & stamp)
{
  PoseWithCovariance estimate = localizer_->estimate();
  if (const auto refined = localizer_->refinedPose()) {
    estimate.pose = *refined;  // the covariance still describes the particle cloud
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = params_.node.global_frame_id;
  pose_msg.pose = toMsg(estimate);
  pose_pub_->publish(pose_msg);

  if (params_.node.publish_particles && particles_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseArray particles_msg;
    particles_msg.header = pose_msg.header;
    const ParticleSet & particles = localizer_->particles();
    particles_msg.poses.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
      particles_msg.poses.push_back(toMsg(particles.pose(i)));
    }
    particles_pub_->publish(particles_msg);
  }

  if (params_.node.publish_tf) {
    publishMapToOdom(stamp, estimate.pose);
  }
}

void Amcl3dNode::publishMapToOdom(const rclcpp::Time & stamp, const Pose & base_in_map)
{
  const auto odom_from_base =
    lookupTransform(params_.node.odom_frame_id, params_.node.base_frame_id, stamp);
  if (!odom_from_base) {
    return;
  }
  // map_from_odom * odom_from_base = map_from_base
  const Eigen::Isometry3d map_from_odom = toIsometry(base_in_map) * odom_from_base->inverse();

  geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(map_from_odom);
  transform.header.stamp = stamp + rclcpp::Duration::from_seconds(params_.node.transform_tolerance);
  transform.header.frame_id = params_.node.global_frame_id;
  transform.child_frame_id = params_.node.odom_frame_id;
  tf_broadcaster_.sendTransform(transform);
}

}  // namespace amcl_3d

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(amcl_3d::Amcl3dNode)
