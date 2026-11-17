#pragma once

#include <rclcpp/node.hpp>
#include <string>

#include "amcl_3d_core/ground_pose_sampler.hpp"
#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/scan_matcher/ndt_scan_matcher.hpp"

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
  /// Take the angular velocity from the IMU instead of the odometry.
  bool use_imu;

  enum class MapType
  {
    kKdTree,
    kDistanceField
  };
  MapType map_type;
  /// CPU threads of the likelihood evaluation (0: all hardware threads).
  size_t map_thread_num;
  /// max_distance is taken from point_cloud_measurement.max_distance.
  DistanceFieldMap::Params distance_field;

  struct GlobalLocalization
  {
    size_t particle_num;
    GroundPoseSampler::Params sampler;
  } global_localization;

  struct ScanMatching
  {
    bool enabled;
    NdtScanMatcher::Params ndt;
  } scan_matching;
};

/// Declares all parameters on `node` and returns their values. Throws std::invalid_argument for
/// invalid values. declareNodeParams() must be called after declareLocalizerParams().
LocalizerParams declareLocalizerParams(rclcpp::Node & node);
NodeParams declareNodeParams(rclcpp::Node & node, const LocalizerParams & localizer_params);

}  // namespace amcl_3d
