#include "amcl_3d_core/scan_matcher/ndt_scan_matcher.hpp"

#include <stdexcept>

namespace amcl_3d
{
NdtScanMatcher::NdtScanMatcher(const PointCloud::ConstPtr & map, const Params & params)
: params_(params)
{
  if (!map || map->empty()) {
    throw std::invalid_argument("NdtScanMatcher: the map point cloud is empty");
  }
  ndt_.setResolution(static_cast<float>(params_.resolution));
  ndt_.setStepSize(params_.step_size);
  ndt_.setTransformationEpsilon(params_.transformation_epsilon);
  ndt_.setMaximumIterations(params_.max_iterations);
  ndt_.setInputTarget(map);
}

ScanMatchingResult NdtScanMatcher::align(
  const std::vector<Eigen::Vector3d> & scan, const Pose & initial_guess)
{
  ScanMatchingResult result;
  result.pose = initial_guess;
  if (scan.empty()) {
    return result;
  }
  PointCloud::Ptr source(new PointCloud);
  source->reserve(scan.size());
  for (const auto & point : scan) {
    source->push_back(pcl::PointXYZ(
      static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())));
  }
  ndt_.setInputSource(source);

  Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
  guess.translation() = initial_guess.position;
  guess.linear() = initial_guess.orientation.toRotationMatrix();
  PointCloud aligned;
  ndt_.align(aligned, guess.matrix().cast<float>());

  const Eigen::Matrix4d transform = ndt_.getFinalTransformation().cast<double>();
  result.pose.position = transform.block<3, 1>(0, 3);
  result.pose.orientation = Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
  result.fitness = ndt_.getFitnessScore();
  result.converged = ndt_.hasConverged() && result.fitness <= params_.max_fitness;
  return result;
}

}  // namespace amcl_3d
