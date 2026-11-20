#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace amcl_3d
{
LikelihoodField::LikelihoodField(std::shared_ptr<const PointCloudMap> map, const Params & params)
: map_(std::move(map)), params_(params)
{
  if (!map_) {
    throw std::invalid_argument("LikelihoodField: map is null");
  }
  if (params_.sigma <= 0.0) {
    throw std::invalid_argument("LikelihoodField: sigma must be positive");
  }
  if (map_->maxDistance() < params_.max_distance) {
    throw std::invalid_argument(
      "LikelihoodField: max_distance exceeds the distance range of the map");
  }
}

Eigen::VectorXd LikelihoodField::logLikelihood(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const
{
  const double inv_two_sigma2 = 1.0 / (2.0 * params_.sigma * params_.sigma);
  return -inv_two_sigma2 * map_->sumClippedSquaredDistances(
                             particles, points, params_.max_distance, params_.thread_num);
}

std::string LikelihoodField::description() const
{
  std::ostringstream out;
  out << "likelihood field (sigma " << params_.sigma << " m, max distance " << params_.max_distance
      << " m) on " << map_->description();
  return out.str();
}

}  // namespace amcl_3d
