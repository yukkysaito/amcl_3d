#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood_factory.hpp"

#include <stdexcept>

namespace amcl_3d
{
void PointCloudLikelihoodParams::setThreadNum(size_t thread_num)
{
  likelihood_field.model.thread_num = thread_num;
  ndt.model.thread_num = thread_num;
  gicp.model.thread_num = thread_num;
}

PointCloudLikelihoodParams::Type toPointCloudLikelihoodType(const std::string & name)
{
  if (name == "likelihood_field") {
    return PointCloudLikelihoodParams::Type::kLikelihoodField;
  }
  if (name == "ndt") {
    return PointCloudLikelihoodParams::Type::kNdt;
  }
  if (name == "gicp") {
    return PointCloudLikelihoodParams::Type::kGicp;
  }
  throw std::invalid_argument(
    "point cloud likelihood type must be 'likelihood_field', 'ndt' or 'gicp': " + name);
}

void validate(const PointCloudLikelihoodParams & params)
{
  switch (params.type) {
    case PointCloudLikelihoodParams::Type::kLikelihoodField:
      validate(params.likelihood_field.map);
      LikelihoodField::validate(params.likelihood_field.model);
      return;
    case PointCloudLikelihoodParams::Type::kNdt:
      GaussianVoxelMap::validate(params.ndt.map);
      NdtLikelihood::validate(params.ndt.model);
      return;
    case PointCloudLikelihoodParams::Type::kGicp:
      GaussianVoxelMap::validate(params.gicp.map);
      GicpLikelihood::validate(params.gicp.model);
      return;
  }
  throw std::invalid_argument("unknown point cloud likelihood type");
}

std::shared_ptr<const PointCloudLikelihood> createPointCloudLikelihood(
  const PointCloud::ConstPtr & cloud, const PointCloudLikelihoodParams & params)
{
  validate(params);
  if (!cloud || cloud->empty()) {
    throw std::invalid_argument("the map point cloud is empty");
  }
  switch (params.type) {
    case PointCloudLikelihoodParams::Type::kNdt:
      return std::make_shared<NdtLikelihood>(
        std::make_shared<GaussianVoxelMap>(*cloud, params.ndt.map), params.ndt.model);
    case PointCloudLikelihoodParams::Type::kGicp:
      return std::make_shared<GicpLikelihood>(
        std::make_shared<GaussianVoxelMap>(*cloud, params.gicp.map), params.gicp.model);
    case PointCloudLikelihoodParams::Type::kLikelihoodField:
      break;
  }
  const auto & field = params.likelihood_field;
  return std::make_shared<LikelihoodField>(
    createPointCloudMap(cloud, field.map, field.model.max_distance), field.model);
}

}  // namespace amcl_3d
