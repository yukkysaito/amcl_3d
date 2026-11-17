#pragma once

#include <rclcpp/node.hpp>
#include <string>

#include "amcl_3d_core/localizer.hpp"

namespace amcl_3d
{
/// Parameters of the ROS wrapper. Algorithm parameters live in LocalizerParams.
struct NodeParams
{
  std::string global_frame_id;
  std::string odom_frame_id;
  std::string base_frame_id;
  /// Map loaded at startup. Empty: wait for the map topic.
  std::string map_pcd_file;
  /// Future dating of the published map->odom transform [s].
  double transform_tolerance;
  /// How long to wait for a transform [s].
  double tf_timeout;
  bool publish_tf;
  bool publish_particles;
  /// Used when the initial pose message has no covariance: x, y, z, roll, pitch, yaw.
  Vector6d default_initial_pose_stddev;
};

/// Declares all parameters on `node` and returns their values. Throws std::invalid_argument for
/// invalid values.
NodeParams declareNodeParams(rclcpp::Node & node);
LocalizerParams declareLocalizerParams(rclcpp::Node & node);

}  // namespace amcl_3d
