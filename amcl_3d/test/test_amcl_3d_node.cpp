// End-to-end tests of the ROS 2 interface with a synthetic map and scans.
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "amcl_3d/amcl_3d_node.hpp"
#include "amcl_3d/conversions.hpp"
#include "amcl_3d_core/map/cuda_distance_field_map.hpp"

namespace amcl_3d
{
namespace
{
using namespace std::chrono_literals;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

/// 8 m x 5 m room with a box and a pillar (no rotational symmetry).
PointCloud::Ptr makeRoomMap()
{
  PointCloud::Ptr map(new PointCloud);
  auto add = [&map](double x, double y, double z) {
    map->push_back(
      pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
  };
  for (double z = 0.0; z <= 2.5; z += 0.1) {
    for (double x = 0.0; x <= 8.0; x += 0.1) {
      add(x, 0.0, z);
      add(x, 5.0, z);
    }
    for (double y = 0.0; y <= 5.0; y += 0.1) {
      add(0.0, y, z);
      add(8.0, y, z);
    }
    if (z <= 1.0) {
      for (double t = 0.0; t <= 1.0; t += 0.1) {
        add(5.0 + t, 3.5, z);
        add(5.0, 3.5 + t, z);
      }
    }
    for (double a = 0.0; a < 2.0 * M_PI; a += 0.3) {
      add(1.5 + 0.3 * std::cos(a), 1.0 + 0.3 * std::sin(a), z);
    }
  }
  for (double x = 0.0; x <= 8.0; x += 0.2) {
    for (double y = 0.0; y <= 5.0; y += 0.2) {
      add(x, y, 0.0);
    }
  }
  return map;
}

sensor_msgs::msg::PointCloud2 toMsg(
  const PointCloud & cloud, const std::string & frame_id, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

geometry_msgs::msg::TransformStamped makeTransform(
  const std::string & parent, const std::string & child,
  const Eigen::Isometry3d & parent_from_child)
{
  geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(parent_from_child);
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  return transform;
}
}  // namespace

/// Runs the node with a test peer node that publishes the map, tf and scans observed from
/// `true_pose_` by a lidar mounted on the vehicle.
class Amcl3dNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    map_ = makeRoomMap();
    true_pose_.position = Eigen::Vector3d(3.0, 2.0, 0.0);
    true_pose_.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    base_from_lidar_ = Eigen::Isometry3d::Identity();
    base_from_lidar_.translation() = Eigen::Vector3d(0.5, 0.0, 1.5);
    base_from_lidar_.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Isometry3d lidar_from_map = (toIsometry(true_pose_) * base_from_lidar_).inverse();
    pcl::transformPointCloud(*map_, scan_, lidar_from_map.matrix().cast<float>());
  }

  void TearDown() override
  {
    if (node_) {
      executor_.remove_node(node_);
    }
    if (io_) {
      executor_.remove_node(io_);
    }
  }

  /// Starts the node under test and the peer, and publishes the map and static tf.
  void start(const std::vector<rclcpp::Parameter> & parameters)
  {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> all = {
      // The synthetic map is grid aligned with 0.1 m spacing, which aliases with coarser
      // distance field voxels (not an issue on real data, see the KITTI evaluation in the README).
      {"point_cloud_likelihood.likelihood_field.map.resolution", 0.1},
      {"random_seed", 1},
      {"initial_particle_num", 300},
      {"scan_sample_num", 200},
      {"scan_filter.voxel_size", 0.2},
      {"motion_model.twist.linear_noise_bias", 0.02},
      {"motion_model.twist.angular_noise_bias", 0.005},
      // scans are published much faster than their stamps advance (0.1 s each)
      {"max_stamp_lead", 10.0},
    };
    all.insert(all.end(), parameters.begin(), parameters.end());
    options.parameter_overrides(all);
    node_ = std::make_shared<Amcl3dNode>(options);
    io_ = rclcpp::Node::make_shared("amcl_3d_test_io");
    executor_.add_node(node_);
    executor_.add_node(io_);

    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(io_);
    static_tf_->sendTransform({
      makeTransform("odom", "base_link", Eigen::Isometry3d::Identity()),
      makeTransform("base_link", "lidar", base_from_lidar_),
    });
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(io_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, io_, false);

    map_pub_ =
      io_->create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).transient_local());
    initial_pose_pub_ = io_->create_publisher<PoseMsg>("initialpose", 1);
    points_pub_ =
      io_->create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    pose_sub_ = io_->create_subscription<PoseMsg>("pose", 10, [this](PoseMsg::SharedPtr msg) {
      last_pose_ = msg;
      ++pose_count_;
    });

    map_pub_->publish(toMsg(*map_, "map", io_->now()));
    spinUntil([] { return false; }, 300ms);  // let the node build the map
  }

  bool spinUntil(const std::function<bool()> & condition, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
      if (condition()) {
        return true;
      }
    }
    return false;
  }

  /// Publishes an initial guess off by (0.3, -0.2) m and -0.1 rad.
  void publishInitialPose()
  {
    PoseMsg initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = io_->now();
    initial_pose.pose.pose.position.x = 3.3;
    initial_pose.pose.pose.position.y = 1.8;
    initial_pose.pose.pose.orientation =
      tf2::toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())));
    initial_pose.pose.covariance[0] = 0.1;
    initial_pose.pose.covariance[7] = 0.1;
    initial_pose.pose.covariance[35] = 0.02;
    const int expected_count = pose_count_ + 1;
    initial_pose_pub_->publish(initial_pose);
    ASSERT_TRUE(spinUntil([&] { return pose_count_ >= expected_count; }, 2s));
  }

  /// Publishes scans stamped 0.1 s apart (a 10 Hz lidar), waiting for the pose of each.
  void publishScans(int scan_num)
  {
    const rclcpp::Time start = io_->now();
    for (int i = 0; i < scan_num; ++i) {
      const int expected_count = pose_count_ + 1;
      points_pub_->publish(
        toMsg(scan_, "lidar", start + rclcpp::Duration::from_seconds(0.1 * (i + 1))));
      ASSERT_TRUE(spinUntil([&] { return pose_count_ >= expected_count; }, 2s))
        << "no pose for scan " << i;
    }
  }

  double positionError() const
  {
    return (fromMsg(last_pose_->pose.pose).position - true_pose_.position).head<2>().norm();
  }
  double orientationError() const
  {
    return fromMsg(last_pose_->pose.pose).orientation.angularDistance(true_pose_.orientation);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<Amcl3dNode> node_;
  rclcpp::Node::SharedPtr io_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<PoseMsg>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  PoseMsg::SharedPtr last_pose_;
  int pose_count_ = 0;

  PointCloud::Ptr map_;
  PointCloud scan_;
  Pose true_pose_;
  Eigen::Isometry3d base_from_lidar_;
};

TEST_F(Amcl3dNodeTest, LocalizesFromScansAndPublishesTf)
{
  start({});
  publishInitialPose();
  publishScans(30);
  EXPECT_EQ(last_pose_->header.frame_id, "map");
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);

  // odom == base_link, so map->odom equals the estimated pose
  ASSERT_TRUE(
    spinUntil([&] { return tf_buffer_->canTransform("map", "odom", tf2::TimePointZero); }, 2s));
  const Eigen::Isometry3d map_from_odom =
    tf2::transformToEigen(tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero));
  EXPECT_LT((map_from_odom.translation() - fromMsg(last_pose_->pose.pose).position).norm(), 1e-6);
}

TEST_F(Amcl3dNodeTest, DropsInputsFromTheFuture)
{
  start({});
  publishInitialPose();
  publishScans(10);
  const builtin_interfaces::msg::Time before = last_pose_->header.stamp;
  // e.g. a sensor on another clock: accepting it would make every later scan stale
  const int count = pose_count_;
  points_pub_->publish(toMsg(scan_, "lidar", io_->now() + rclcpp::Duration::from_seconds(3600.0)));
  spinUntil([] { return false; }, 300ms);
  EXPECT_EQ(pose_count_, count);
  EXPECT_EQ(last_pose_->header.stamp, before);
  publishScans(10);  // still localizing
  EXPECT_LT(positionError(), 0.1);
}

TEST_F(Amcl3dNodeTest, RestartsWhenTimeJumpsBack)
{
  start({});
  publishInitialPose();
  publishScans(20);
  ASSERT_LT(positionError(), 0.1);
  // a looping rosbag: the scans start over 60 s earlier
  const rclcpp::Time start = io_->now() - rclcpp::Duration::from_seconds(60.0);
  for (int i = 0; i < 10; ++i) {
    const rclcpp::Time stamp = start + rclcpp::Duration::from_seconds(0.1 * (i + 1));
    const int expected_count = pose_count_ + 1;
    points_pub_->publish(toMsg(scan_, "lidar", stamp));
    ASSERT_TRUE(spinUntil([&] { return pose_count_ >= expected_count; }, 2s)) << "scan " << i;
    EXPECT_EQ(rclcpp::Time(last_pose_->header.stamp), stamp);
  }
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

class Amcl3dNodeDelayTest : public Amcl3dNodeTest, public ::testing::WithParamInterface<bool>
{
};

TEST_P(Amcl3dNodeDelayTest, FusesDelayedScansAndPublishesTheNewestEstimate)
{
  start({{"scan_tempered_smc.enabled", GetParam()}});
  publishInitialPose();
  const rclcpp::Time start = io_->now();
  rclcpp::Time newest = start;
  for (int i = 0; i < 30; ++i) {
    // swap every other pair: the scan of 0.2 s arrives after the one of 0.3 s, and so on
    const int slot = i % 4 == 1 ? i + 1 : (i % 4 == 2 ? i - 1 : i);
    const rclcpp::Time stamp = start + rclcpp::Duration::from_seconds(0.1 * (slot + 1));
    newest = std::max(newest, stamp);
    const int expected_count = pose_count_ + 1;
    points_pub_->publish(toMsg(scan_, "lidar", stamp));
    ASSERT_TRUE(spinUntil([&] { return pose_count_ >= expected_count; }, 2s)) << "scan " << i;
    EXPECT_EQ(rclcpp::Time(last_pose_->header.stamp), newest) << "scan " << i;
  }
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

// replayed with one importance update per scan and with tempered SMC
INSTANTIATE_TEST_SUITE_P(Updates, Amcl3dNodeDelayTest, ::testing::Bool());

TEST_F(Amcl3dNodeTest, GlobalLocalizationService)
{
  start({
    {"resampling.type", "kld"},
    {"resampling.kld.max_particle_num", 20000},
    {"global_localization.particle_num", 20000},
    {"global_localization.cell_size", 0.5},
    {"scan_sample_num", 100},
  });
  auto client = io_->create_client<std_srvs::srv::Trigger>("/amcl_3d/global_localization");
  ASSERT_TRUE(client->wait_for_service(2s));
  auto future =
    client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
  ASSERT_TRUE(spinUntil([&] { return future.wait_for(0s) == std::future_status::ready; }, 2s));
  const auto response = future.get();
  ASSERT_TRUE(response->success) << response->message;

  // the same static scan over and over: the tempered likelihood (scan_max_effective_points) needs
  // about 100 of them to settle from the whole map
  publishScans(100);
  // the distance field trades ~1-2 cm of accuracy for speed (see DistanceFieldMap)
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.5 * M_PI / 180.0);
}

TEST_F(Amcl3dNodeTest, ScanMatchingRefinesPublishedPose)
{
  // The synthetic scans are noise free, so the untempered likelihood is the right model here; with
  // tempering the particle estimate wanders by a few cm at standstill, and NDT then may settle in
  // a local minimum of the grid aligned map. This test is about the refinement.
  start({{"scan_matching.enabled", true}, {"scan_max_effective_points", 0}});
  publishInitialPose();
  publishScans(20);
  // tighter than the particle filter alone (0.1 m, 1 deg in LocalizesFromScansAndPublishesTf)
  EXPECT_LT(positionError(), 0.02);
  EXPECT_LT(orientationError(), 0.5 * M_PI / 180.0);
}

TEST_F(Amcl3dNodeTest, LocalizesWithTemperedSmc)
{
  start({{"scan_tempered_smc.enabled", true}, {"scan_max_effective_points", 100}});
  publishInitialPose();
  publishScans(20);
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

TEST_F(Amcl3dNodeTest, LocalizesWithGaussNewtonMoves)
{
  start(
    {{"scan_tempered_smc.enabled", true},
     {"scan_tempered_smc.kernel", "gauss_newton"},
     {"scan_tempered_smc.cess_ratio", 0.15},
     {"scan_max_effective_points", 100}});
  publishInitialPose();
  publishScans(20);
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

TEST_F(Amcl3dNodeTest, LocalizesWithVerticalGaussNewtonMovesFromCoarseToFine)
{
  start(
    {{"scan_tempered_smc.enabled", true},
     {"scan_tempered_smc.kernel", "vertical_gauss_newton"},
     {"scan_tempered_smc.coarse_to_fine", true},
     {"scan_tempered_smc.cess_ratio", 0.15},
     {"scan_max_effective_points", 100}});
  publishInitialPose();
  publishScans(20);
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

TEST_F(Amcl3dNodeTest, RejectsUnknownMoveKernel)
{
  rclcpp::NodeOptions unknown_kernel;
  unknown_kernel.parameter_overrides({{"scan_tempered_smc.kernel", "hmc"}});
  EXPECT_THROW(Amcl3dNode{unknown_kernel}, std::invalid_argument);
}

TEST_F(Amcl3dNodeTest, LocalizesWithNdtLikelihood)
{
  start({{"point_cloud_likelihood.type", "ndt"}});
  publishInitialPose();
  publishScans(60);  // the coarse voxel distributions settle more slowly on a repeated scan
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

TEST_F(Amcl3dNodeTest, RejectsUnknownLikelihoodAtStartup)
{
  for (const std::string type : {"icp", "gicp"}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"point_cloud_likelihood.type", type}});
    EXPECT_THROW(Amcl3dNode{options}, std::invalid_argument) << type;
  }
}

TEST_F(Amcl3dNodeTest, LocalizesWithCuda)
{
  if (!CudaDistanceFieldMap::available()) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"point_cloud_likelihood.likelihood_field.map.device", "cuda"}});
    EXPECT_THROW(Amcl3dNode{options}, std::invalid_argument);  // reported at startup
    GTEST_SKIP() << "no CUDA device (or built without CUDA)";
  }
  start(
    {{"point_cloud_likelihood.likelihood_field.map.device", "cuda"},
     {"initial_particle_num", 3000}});
  publishInitialPose();
  publishScans(30);
  EXPECT_LT(positionError(), 0.1);
  EXPECT_LT(orientationError(), 1.0 * M_PI / 180.0);
}

}  // namespace amcl_3d
