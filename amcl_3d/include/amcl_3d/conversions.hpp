// Conversions between ROS messages and amcl_3d_core types.
#pragma once

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
Time fromMsg(const builtin_interfaces::msg::Time & msg);

Pose fromMsg(const geometry_msgs::msg::Pose & msg);
geometry_msgs::msg::Pose toMsg(const Pose & pose);

PoseWithCovariance fromMsg(const geometry_msgs::msg::PoseWithCovariance & msg);
geometry_msgs::msg::PoseWithCovariance toMsg(const PoseWithCovariance & pose);

Twist fromMsg(const geometry_msgs::msg::Twist & msg);

Eigen::Isometry3d toIsometry(const Pose & pose);
Pose toPose(const Eigen::Isometry3d & isometry);

/// Expresses `pose` (given in frame A) in frame B, where `b_from_a` maps points from A to B.
/// The covariance is rotated accordingly.
PoseWithCovariance transform(const PoseWithCovariance & pose, const Eigen::Isometry3d & b_from_a);

}  // namespace amcl_3d
