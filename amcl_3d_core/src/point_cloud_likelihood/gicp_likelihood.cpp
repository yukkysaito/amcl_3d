#include "amcl_3d_core/point_cloud_likelihood/gicp_likelihood.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Cholesky>
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "parallel_for.hpp"

namespace amcl_3d
{
namespace
{
/// One scan with the covariance of each sampled point, computed once.
class GicpMeasurementModel : public MeasurementModel
{
public:
  GicpMeasurementModel(
    std::shared_ptr<const GicpLikelihood> likelihood, std::vector<Eigen::Vector3d> points,
    std::vector<Eigen::Matrix3d> covariances)
  : likelihood_(std::move(likelihood)),
    points_(std::move(points)),
    covariances_(std::move(covariances))
  {
  }

  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
  {
    LogLikelihood result;
    result.values = likelihood_->logLikelihood(particles, points_, covariances_);
    result.observation_num = points_.size();
    return result;
  }

private:
  std::shared_ptr<const GicpLikelihood> likelihood_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Matrix3d> covariances_;
};
}  // namespace

void GicpLikelihood::validate(const Params & params)
{
  if (
    !(params.sigma > 0.0) || !(params.max_mahalanobis_distance > 0.0) || params.neighbor_num < 3) {
    throw std::invalid_argument(
      "GicpLikelihood: sigma and max_mahalanobis_distance must be positive, neighbor_num >= 3");
  }
}

GicpLikelihood::GicpLikelihood(std::shared_ptr<const GaussianVoxelMap> map, const Params & params)
: map_(std::move(map)), params_(params)
{
  if (!map_) {
    throw std::invalid_argument("GicpLikelihood: map is null");
  }
  validate(params_);
  max_squared_distance_ = params_.max_mahalanobis_distance * params_.max_mahalanobis_distance;
  const Eigen::Matrix3d noise = params_.sigma * params_.sigma * Eigen::Matrix3d::Identity();
  information_.reserve(map_->voxels().size());
  for (const auto & voxel : map_->voxels()) {
    information_.push_back(
      Eigen::LLT<Eigen::Matrix3d>(voxel.covariance + noise).solve(Eigen::Matrix3d::Identity()));
  }
}

std::vector<Eigen::Matrix3d> GicpLikelihood::estimateCovariances(
  const std::vector<Eigen::Vector3d> & scan, const std::vector<Eigen::Vector3d> & points) const
{
  std::vector<Eigen::Matrix3d> covariances(points.size(), Eigen::Matrix3d::Zero());
  if (scan.size() < 3) {
    return covariances;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(scan.size());
  for (const auto & p : scan) {
    cloud->push_back(pcl::PointXYZ(
      static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  const int k = static_cast<int>(std::min(params_.neighbor_num, scan.size()));
  std::vector<int> indices(static_cast<size_t>(k));
  std::vector<float> squared_distances(static_cast<size_t>(k));
  for (size_t i = 0; i < points.size(); ++i) {
    const pcl::PointXYZ query(
      static_cast<float>(points[i].x()), static_cast<float>(points[i].y()),
      static_cast<float>(points[i].z()));
    const int found = tree.nearestKSearch(query, k, indices, squared_distances);
    if (found < 3) {
      continue;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int n = 0; n < found; ++n) {
      mean += scan[static_cast<size_t>(indices[static_cast<size_t>(n)])];
    }
    mean /= found;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int n = 0; n < found; ++n) {
      const Eigen::Vector3d d = scan[static_cast<size_t>(indices[static_cast<size_t>(n)])] - mean;
      covariance += d * d.transpose();
    }
    covariances[i] = covariance / found;
  }
  return covariances;
}

double GicpLikelihood::scorePoint(
  const Eigen::Vector3d & point, const Eigen::Matrix3d & covariance) const
{
  // correspondence: the voxel explaining the point best by the map distribution alone
  const auto & voxels = map_->voxels();
  int64_t best_voxel = -1;
  double best_map_score = max_squared_distance_;
  for (const uint32_t v : map_->candidates(point)) {
    const Eigen::Vector3d error = point - voxels[v].mean;
    const double score = error.dot(information_[v] * error);
    if (score < best_map_score) {
      best_map_score = score;
      best_voxel = v;
    }
  }
  if (best_voxel < 0) {
    return max_squared_distance_;
  }
  const auto & voxel = voxels[static_cast<size_t>(best_voxel)];
  const Eigen::Matrix3d combined =
    voxel.covariance + covariance + params_.sigma * params_.sigma * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d error = point - voxel.mean;
  return std::min(error.dot(combined.inverse() * error), max_squared_distance_);
}

Eigen::VectorXd GicpLikelihood::logLikelihood(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points,
  const std::vector<Eigen::Matrix3d> & covariances) const
{
  if (covariances.size() != points.size()) {
    throw std::invalid_argument("GicpLikelihood: one covariance per point is required");
  }
  Eigen::VectorXd values(static_cast<Eigen::Index>(particles.size()));
  parallelFor(values.size(), params_.thread_num, [&](int64_t i) {
    const Pose pose = particles.pose(static_cast<size_t>(i));
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    double sum = 0.0;
    for (size_t k = 0; k < points.size(); ++k) {
      sum += scorePoint(
        rotation * points[k] + pose.position, rotation * covariances[k] * rotation.transpose());
    }
    values(i) = -0.5 * sum;
  });
  return values;
}

Eigen::VectorXd GicpLikelihood::logLikelihood(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const
{
  return logLikelihood(particles, points, estimateCovariances(points, points));
}

std::unique_ptr<MeasurementModel> GicpLikelihood::createMeasurementModel(
  const std::vector<Eigen::Vector3d> & scan, std::vector<Eigen::Vector3d> sampled) const
{
  std::vector<Eigen::Matrix3d> covariances = estimateCovariances(scan, sampled);
  return std::make_unique<GicpMeasurementModel>(
    std::static_pointer_cast<const GicpLikelihood>(shared_from_this()), std::move(sampled),
    std::move(covariances));
}

std::string GicpLikelihood::description() const
{
  std::ostringstream out;
  out << "GICP (sigma " << params_.sigma << " m, max Mahalanobis distance "
      << params_.max_mahalanobis_distance << ", " << params_.neighbor_num
      << " neighbors per scan covariance) on " << map_->description();
  return out.str();
}

}  // namespace amcl_3d
