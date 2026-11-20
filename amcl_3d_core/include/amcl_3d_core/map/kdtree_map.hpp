#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include "amcl_3d_core/map/point_cloud_map.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Exact nearest neighbor search on the map points with a k-d tree.
class KdTreeMap : public PointCloudMap
{
public:
  explicit KdTreeMap(const PointCloud::ConstPtr & cloud);

  double squaredDistanceToNearest(const Eigen::Vector3d & point) const override;
  std::string description() const override;

private:
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

}  // namespace amcl_3d
