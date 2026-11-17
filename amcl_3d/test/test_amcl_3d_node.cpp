// End-to-end test of the ROS 2 interface with a synthetic map and scans.
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <functional>
#include <memory>

#include "amcl_3d/amcl_3d_node.hpp"
#include "amcl_3d/conversions.hpp"

namespace amcl_3d
{
namespace
{
using namespace std::chrono_literals;

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

class Amcl3dNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

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

  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(Amcl3dNodeTest, LocalizesFromScansAndPublishesTf)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"random_seed", 1},
    {"initial_particle_num", 300},
    {"scan_sample_num", 200},
    {"motion_model.linear_noise_bias", 0.02},
    {"motion_model.angular_noise_bias", 0.005},
  });
  auto node = std::make_shared<Amcl3dNode>(options);
  auto io = rclcpp::Node::make_shared("amcl_3d_test_io");
  executor_.add_node(node);
  executor_.add_node(io);

  // tf: odom -> base_link identity, base_link -> lidar mounted 1.5 m above, rotated by 90 deg
  Eigen::Isometry3d base_from_lidar = Eigen::Isometry3d::Identity();
  base_from_lidar.translation() = Eigen::Vector3d(0.5, 0.0, 1.5);
  base_from_lidar.linear() =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  tf2_ros::StaticTransformBroadcaster static_tf(io);
  static_tf.sendTransform({
    makeTransform("odom", "base_link", Eigen::Isometry3d::Identity()),
    makeTransform("base_link", "lidar", base_from_lidar),
  });

  auto map_pub =
    io->create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).transient_local());
  auto initial_pose_pub =
    io->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1);
  auto points_pub =
    io->create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose;
  int pose_count = 0;
  auto pose_sub = io->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose", 10, [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
      last_pose = msg;
      ++pose_count;
    });

  const PointCloud::Ptr map = makeRoomMap();
  map_pub->publish(toMsg(*map, "map", io->now()));
  spinUntil([] { return false; }, 300ms);

  // initial guess off by (0.3, -0.2) m and -0.1 rad
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = io->now();
  initial_pose.pose.pose.position.x = 3.3;
  initial_pose.pose.pose.position.y = 1.8;
  initial_pose.pose.pose.orientation =
    tf2::toMsg(Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())));
  initial_pose.pose.covariance[0] = 0.1;
  initial_pose.pose.covariance[7] = 0.1;
  initial_pose.pose.covariance[35] = 0.02;
  initial_pose_pub->publish(initial_pose);
  ASSERT_TRUE(spinUntil([&] { return pose_count > 0; }, 2s)) << "no pose after initialization";

  // scans observed from the true pose, expressed in the lidar frame
  Pose true_pose;
  true_pose.position = Eigen::Vector3d(3.0, 2.0, 0.0);
  true_pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Isometry3d lidar_from_map = (toIsometry(true_pose) * base_from_lidar).inverse();
  PointCloud scan;
  pcl::transformPointCloud(*map, scan, lidar_from_map.matrix().cast<float>());

  tf2_ros::Buffer tf_buffer(io->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer, io, false);
  for (int i = 0; i < 30; ++i) {
    const int expected_count = pose_count + 1;
    points_pub->publish(toMsg(scan, "lidar", io->now()));
    ASSERT_TRUE(spinUntil([&] { return pose_count >= expected_count; }, 2s))
      << "no pose for scan " << i;
  }

  const Pose estimate = fromMsg(last_pose->pose.pose);
  EXPECT_EQ(last_pose->header.frame_id, "map");
  EXPECT_LT((estimate.position - true_pose.position).head<2>().norm(), 0.1);
  EXPECT_LT(estimate.orientation.angularDistance(true_pose.orientation), 1.0 * M_PI / 180.0);

  // odom == base_link, so map->odom equals the estimated pose
  ASSERT_TRUE(
    spinUntil([&] { return tf_buffer.canTransform("map", "odom", tf2::TimePointZero); }, 2s));
  const Eigen::Isometry3d map_from_odom =
    tf2::transformToEigen(tf_buffer.lookupTransform("map", "odom", tf2::TimePointZero));
  EXPECT_LT((map_from_odom.translation() - estimate.position).norm(), 1e-6);
}

}  // namespace amcl_3d
