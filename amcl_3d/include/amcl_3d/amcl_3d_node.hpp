#pragma once

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "amcl_3d/parameters.hpp"
#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/pose_sampler/ground_pose_sampler.hpp"

namespace amcl_3d
{
/// ROS 2 interface of the 3D Monte Carlo localization.
///
/// Subscriptions:
///   map         (sensor_msgs/PointCloud2, transient local) map points
///   initialpose (geometry_msgs/PoseWithCovarianceStamped)  (re)initializes the particles
///   odom        (nav_msgs/Odometry)                        twist used for prediction
///   imu         (sensor_msgs/Imu)                          angular velocity, if use_imu
///   points      (sensor_msgs/PointCloud2)                  LiDAR scan used for correction
/// Publications:
///   pose        (geometry_msgs/PoseWithCovarianceStamped)  estimate in the global frame
///                                                          (refined by scan matching if enabled)
///   particles   (geometry_msgs/PoseArray)
///   tf          global_frame -> odom_frame
/// Services:
///   ~/global_localization (std_srvs/Trigger)              spreads particles over the whole map
///
/// All callbacks share the default mutually exclusive callback group, so the localizer is never
/// accessed concurrently.
class Amcl3dNode : public rclcpp::Node
{
public:
  explicit Amcl3dNode(const rclcpp::NodeOptions & options);

private:
  void onMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void onGlobalLocalization(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /// Builds everything derived from the map: the measurement map, the global localization
  /// sampler and the scan matcher.
  void setMap(const PointCloud::Ptr & map);
  void addTwist(const Twist & twist, const builtin_interfaces::msg::Time & stamp);
  void publishEstimate(const rclcpp::Time & stamp);
  void publishMapToOdom(const rclcpp::Time & stamp, const Pose & base_in_map);

  /// Transform mapping points from `source_frame` to `target_frame` at `stamp` (zero: latest).
  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & stamp);

  // declaration order matters: parameters are declared before the localizer is built
  LocalizerParams localizer_params_;
  NodeParams params_;
  std::unique_ptr<Localizer> localizer_;
  std::unique_ptr<GroundPoseSampler> global_sampler_;

  std::optional<Eigen::Vector3d> odom_linear_velocity_;  // latest, used with the IMU
  std::optional<Eigen::Quaterniond> base_from_imu_;      // static, looked up once
  size_t reported_expansion_resets_ = 0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_localization_srv_;
};

}  // namespace amcl_3d
