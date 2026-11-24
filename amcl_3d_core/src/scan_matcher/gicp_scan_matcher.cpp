#include "amcl_3d_core/scan_matcher/gicp_scan_matcher.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "amcl_3d_core/math.hpp"
#include "parallel_for.hpp"

namespace amcl_3d
{
namespace
{
using Matrix6d = Eigen::Matrix<double, 6, 6>;

Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

pcl::PointXYZ toPoint(const Eigen::Vector3d & p)
{
  return pcl::PointXYZ(
    static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
}

/// Plane covariance of `point` from its nearest neighbors in the tree's cloud; identity if there
/// are too few.
Eigen::Matrix3d neighborhoodCovariance(
  const pcl::KdTreeFLANN<pcl::PointXYZ> & tree, const pcl::PointXYZ & point, int k)
{
  std::vector<int> indices(static_cast<size_t>(k));
  std::vector<float> squared_distances(static_cast<size_t>(k));
  const int found = tree.nearestKSearch(point, k, indices, squared_distances);
  if (found < 3) {
    return Eigen::Matrix3d::Identity();
  }
  const auto & cloud = *tree.getInputCloud();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int n = 0; n < found; ++n) {
    mean +=
      cloud[static_cast<size_t>(indices[static_cast<size_t>(n)])].getVector3fMap().cast<double>();
  }
  mean /= found;
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (int n = 0; n < found; ++n) {
    const Eigen::Vector3d d =
      cloud[static_cast<size_t>(indices[static_cast<size_t>(n)])].getVector3fMap().cast<double>() -
      mean;
    covariance += d * d.transpose();
  }
  return regularizeToPlane(covariance / found);
}

/// Normal equations of one Gauss-Newton step, summed over a chunk of scan points.
struct Accumulator
{
  Matrix6d H = Matrix6d::Zero();
  Vector6d b = Vector6d::Zero();
  double error = 0.0;
  size_t inlier_num = 0;
};
}  // namespace

Eigen::Matrix3d regularizeToPlane(const Eigen::Matrix3d & covariance)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  // eigenvalues in ascending order: the first axis is the surface normal
  const Eigen::Vector3d values(1e-3, 1.0, 1.0);
  return solver.eigenvectors() * values.asDiagonal() * solver.eigenvectors().transpose();
}

std::vector<Eigen::Matrix3d> planeCovariances(
  const std::vector<Eigen::Vector3d> & points, size_t neighbor_num, size_t thread_num)
{
  std::vector<Eigen::Matrix3d> covariances(points.size(), Eigen::Matrix3d::Identity());
  if (points.size() < 3) {
    return covariances;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(points.size());
  for (const auto & p : points) {
    cloud->push_back(toPoint(p));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  const int k = static_cast<int>(std::min(neighbor_num, points.size()));
  parallelFor(static_cast<int64_t>(points.size()), thread_num, [&](int64_t i) {
    covariances[static_cast<size_t>(i)] =
      neighborhoodCovariance(tree, cloud->points[static_cast<size_t>(i)], k);
  });
  return covariances;
}

GicpScanMatcher::GicpScanMatcher(const PointCloud::ConstPtr & map, const Params & params)
: params_(params), map_(map)
{
  if (!map_ || map_->size() < 3) {
    throw std::invalid_argument("GicpScanMatcher: the map point cloud is empty");
  }
  if (
    !(params_.max_correspondence_distance > 0.0) || params_.neighbor_num < 3 ||
    params_.max_iterations < 1 || !(params_.translation_epsilon > 0.0) ||
    !(params_.rotation_epsilon > 0.0) || !(params_.robust_kernel_width > 0.0) ||
    !(params_.min_inlier_ratio >= 0.0) || !(params_.min_inlier_ratio <= 1.0)) {
    throw std::invalid_argument("GicpScanMatcher: invalid parameters");
  }
  tree_.setInputCloud(map_);
  map_covariances_.resize(map_->size());
  const int k = static_cast<int>(std::min(params_.neighbor_num, map_->size()));
  parallelFor(static_cast<int64_t>(map_->size()), params_.thread_num, [&](int64_t i) {
    const Eigen::Matrix3d c =
      neighborhoodCovariance(tree_, map_->points[static_cast<size_t>(i)], k);
    map_covariances_[static_cast<size_t>(i)] = {
      static_cast<float>(c(0, 0)), static_cast<float>(c(0, 1)), static_cast<float>(c(0, 2)),
      static_cast<float>(c(1, 1)), static_cast<float>(c(1, 2)), static_cast<float>(c(2, 2))};
  });
}

ScanMatchingResult GicpScanMatcher::align(
  const std::vector<Eigen::Vector3d> & scan, const Pose & initial_guess)
{
  ScanMatchingResult result;
  result.pose = initial_guess;
  if (scan.size() < 3) {
    return result;
  }
  const std::vector<Eigen::Matrix3d> scan_covariances =
    planeCovariances(scan, params_.neighbor_num, params_.thread_num);
  const double kernel_squared = params_.robust_kernel_width * params_.robust_kernel_width;
  const float max_squared_distance =
    static_cast<float>(params_.max_correspondence_distance * params_.max_correspondence_distance);

  Eigen::Matrix3d rotation = initial_guess.orientation.toRotationMatrix();
  Eigen::Vector3d translation = initial_guess.position;
  constexpr int64_t kChunk = 128;
  const int64_t chunk_num = (static_cast<int64_t>(scan.size()) + kChunk - 1) / kChunk;
  std::vector<Accumulator> chunks(static_cast<size_t>(chunk_num));
  Accumulator total;
  bool converged = false;
  for (int iteration = 0; iteration < params_.max_iterations && !converged; ++iteration) {
    parallelFor(chunk_num, params_.thread_num, [&](int64_t c) {
      Accumulator & acc = chunks[static_cast<size_t>(c)];
      acc = Accumulator{};
      std::vector<int> index(1);
      std::vector<float> squared_distance(1);
      const size_t end = std::min(scan.size(), static_cast<size_t>((c + 1) * kChunk));
      for (size_t k = static_cast<size_t>(c * kChunk); k < end; ++k) {
        const Eigen::Vector3d q = rotation * scan[k] + translation;
        if (
          tree_.nearestKSearch(toPoint(q), 1, index, squared_distance) < 1 ||
          squared_distance[0] > max_squared_distance) {
          continue;
        }
        const size_t m = static_cast<size_t>(index[0]);
        const auto & u = map_covariances_[m];
        Eigen::Matrix3d map_covariance;
        map_covariance << u[0], u[1], u[2], u[1], u[3], u[4], u[2], u[4], u[5];
        const Eigen::Matrix3d information =
          (map_covariance + rotation * scan_covariances[k] * rotation.transpose()).inverse();
        const Eigen::Vector3d e = q - map_->points[m].getVector3fMap().cast<double>();
        // left perturbation exp(delta) T, delta = (rotation, translation): de = -[q]x w + dt
        Eigen::Matrix<double, 3, 6> J;
        J.leftCols<3>() = -skew(q);
        J.rightCols<3>().setIdentity();
        const double squared_mahalanobis = e.dot(information * e);
        // Cauchy weight (iteratively reweighted least squares)
        const double w = 1.0 / (1.0 + squared_mahalanobis / kernel_squared);
        const Eigen::Matrix<double, 6, 3> JtI = w * J.transpose() * information;
        acc.H += JtI * J;
        acc.b += JtI * e;
        acc.error += squared_mahalanobis;
        ++acc.inlier_num;
      }
    });
    total = Accumulator{};
    for (const auto & chunk : chunks) {
      total.H += chunk.H;
      total.b += chunk.b;
      total.error += chunk.error;
      total.inlier_num += chunk.inlier_num;
    }
    if (total.inlier_num < 6) {
      return result;
    }
    const Vector6d delta = total.H.ldlt().solve(-total.b);
    const Eigen::Matrix3d step_rotation = expMap(delta.head<3>()).toRotationMatrix();
    rotation = step_rotation * rotation;
    translation = step_rotation * translation + delta.tail<3>();
    converged = delta.head<3>().norm() < params_.rotation_epsilon &&
                delta.tail<3>().norm() < params_.translation_epsilon;
  }

  result.pose.position = translation;
  result.pose.orientation = Eigen::Quaterniond(rotation).normalized();
  result.fitness = total.error / static_cast<double>(total.inlier_num);
  const double inlier_ratio =
    static_cast<double>(total.inlier_num) / static_cast<double>(scan.size());
  result.converged = converged && inlier_ratio >= params_.min_inlier_ratio;
  return result;
}

std::string GicpScanMatcher::description() const
{
  std::ostringstream out;
  out << "GICP scan matching (" << map_->size() << " map points, " << params_.neighbor_num
      << " neighbors per covariance)";
  return out.str();
}

}  // namespace amcl_3d
