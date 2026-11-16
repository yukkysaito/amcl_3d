#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "amcl_3d/parameters.hpp"
#include "amcl_3d_core/localizer.hpp"

namespace amcl_3d
{
/// ROS 2 interface of the 3D Monte Carlo localization.
///
/// Subscriptions:
///   map         (sensor_msgs/PointCloud2, transient local) map points
///   initialpose (geometry_msgs/PoseWithCovarianceStamped)  (re)initializes the particles
///   odom        (nav_msgs/Odometry)                        twist used for prediction
///   points      (sensor_msgs/PointCloud2)                  LiDAR scan used for correction
/// Publications:
///   pose        (geometry_msgs/PoseWithCovarianceStamped)  estimate in the global frame
///   particles   (geometry_msgs/PoseArray)
///   tf          global_frame -> odom_frame
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
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  void setMap(const PointCloud::Ptr & map);
  void publishEstimate(const rclcpp::Time & stamp);
  void publishMapToOdom(const rclcpp::Time & stamp, const Pose & base_in_map);

  /// Transform mapping points from `source_frame` to `target_frame` at `stamp` (zero: latest).
  std::optional<Eigen::Isometry3d> lookupTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & stamp);

  NodeParams params_;
  std::unique_ptr<Localizer> localizer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
};

}  // namespace amcl_3d
