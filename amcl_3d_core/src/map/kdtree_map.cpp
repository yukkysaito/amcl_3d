#include "amcl_3d_core/map/kdtree_map.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

namespace amcl_3d
{
KdTreeMap::KdTreeMap(const PointCloud::ConstPtr & cloud)
{
  if (!cloud || cloud->empty()) {
    throw std::invalid_argument("KdTreeMap: the map point cloud is empty");
  }
  kdtree_.setInputCloud(cloud);
}

double KdTreeMap::squaredDistanceToNearest(const Eigen::Vector3d & point) const
{
  const pcl::PointXYZ query(
    static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()));
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  if (kdtree_.nearestKSearch(query, 1, indices, squared_distances) <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(squared_distances.front());
}

}  // namespace amcl_3d
