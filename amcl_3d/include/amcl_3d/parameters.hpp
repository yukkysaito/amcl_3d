#pragma once

#include <rclcpp/node.hpp>
#include <string>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood_factory.hpp"
#include "amcl_3d_core/pose_sampler/ground_pose_sampler.hpp"
#include "amcl_3d_core/scan_matcher/gicp_scan_matcher.hpp"
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

  /// Scan likelihood and the map representation it is built on.
  PointCloudLikelihoodParams point_cloud_likelihood;

  struct GlobalLocalization
  {
    size_t particle_num;
    GroundPoseSampler::Params sampler;
  } global_localization;

  struct ScanMatching
  {
    bool enabled;
    std::string type;  // "gicp" or "ndt"
    GicpScanMatcher::Params gicp;
    NdtScanMatcher::Params ndt;
  } scan_matching;
};

struct Amcl3dParams
{
  LocalizerParams localizer;
  NodeParams node;
};

/// Declares all parameters on `node` and returns their values. Throws std::invalid_argument for
/// invalid values.
Amcl3dParams declareParams(rclcpp::Node & node);

}  // namespace amcl_3d
