#include "amcl_3d/conversions.hpp"

#include <rclcpp/time.hpp>

namespace amcl_3d
{
namespace
{
// geometry_msgs covariance is a row-major 6x6 array.
using RowMajorMatrix6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
}  // namespace

Time fromMsg(const builtin_interfaces::msg::Time & msg)
{
  return Time{rclcpp::Time(msg).nanoseconds()};
}

Pose fromMsg(const geometry_msgs::msg::Pose & msg)
{
  Pose pose;
  pose.position = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  pose.orientation =
    Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
      .normalized();
  return pose;
}

geometry_msgs::msg::Pose toMsg(const Pose & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.position.x();
  msg.position.y = pose.position.y();
  msg.position.z = pose.position.z();
  msg.orientation.x = pose.orientation.x();
  msg.orientation.y = pose.orientation.y();
  msg.orientation.z = pose.orientation.z();
  msg.orientation.w = pose.orientation.w();
  return msg;
}

PoseWithCovariance fromMsg(const geometry_msgs::msg::PoseWithCovariance & msg)
{
  PoseWithCovariance pose;
  pose.pose = fromMsg(msg.pose);
  pose.covariance = Eigen::Map<const RowMajorMatrix6d>(msg.covariance.data());
  return pose;
}

geometry_msgs::msg::PoseWithCovariance toMsg(const PoseWithCovariance & pose)
{
  geometry_msgs::msg::PoseWithCovariance msg;
  msg.pose = toMsg(pose.pose);
  Eigen::Map<RowMajorMatrix6d>(msg.covariance.data()) = pose.covariance;
  return msg;
}

Twist fromMsg(const geometry_msgs::msg::Twist & msg)
{
  Twist twist;
  twist.linear = Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
  twist.angular = Eigen::Vector3d(msg.angular.x, msg.angular.y, msg.angular.z);
  return twist;
}

Eigen::Isometry3d toIsometry(const Pose & pose)
{
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = pose.position;
  isometry.linear() = pose.orientation.toRotationMatrix();
  return isometry;
}

Pose toPose(const Eigen::Isometry3d & isometry)
{
  Pose pose;
  pose.position = isometry.translation();
  pose.orientation = Eigen::Quaterniond(isometry.rotation()).normalized();
  return pose;
}

PoseWithCovariance transform(const PoseWithCovariance & pose, const Eigen::Isometry3d & b_from_a)
{
  PoseWithCovariance transformed;
  transformed.pose = toPose(b_from_a * toIsometry(pose.pose));
  Matrix6d jacobian = Matrix6d::Zero();
  jacobian.topLeftCorner<3, 3>() = b_from_a.rotation();
  jacobian.bottomRightCorner<3, 3>() = b_from_a.rotation();
  transformed.covariance = jacobian * pose.covariance * jacobian.transpose();
  return transformed;
}

}  // namespace amcl_3d
