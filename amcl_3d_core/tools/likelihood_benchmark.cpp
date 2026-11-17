// Measures the cost of one likelihood evaluation (the hot spot of the filter) for different
// particle counts, scan point counts and map representations, on a synthetic map.
//
//   likelihood_benchmark [repetitions=5] [threads=4 (0: all)]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <vector>

#include "amcl_3d_core/map/cuda_distance_field_map.hpp"
#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"
#include "amcl_3d_core/particle_filter.hpp"

using namespace amcl_3d;

namespace
{
/// 40 m x 40 m area: ground, walls every 10 m and random poles (~250k points).
PointCloud::Ptr makeMap()
{
  PointCloud::Ptr map(new PointCloud);
  auto add = [&map](double x, double y, double z) {
    map->push_back(
      pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
  };
  for (double x = 0.0; x <= 40.0; x += 0.2) {
    for (double y = 0.0; y <= 40.0; y += 0.2) {
      add(x, y, 0.0);
    }
  }
  for (double wall = 0.0; wall <= 40.0; wall += 10.0) {
    for (double t = 0.0; t <= 40.0; t += 0.1) {
      for (double z = 0.0; z <= 3.0; z += 0.1) {
        add(wall, t, z);
        add(t, wall, z);
      }
    }
  }
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> uniform(0.0, 40.0);
  for (int pole = 0; pole < 100; ++pole) {
    const double x = uniform(rng), y = uniform(rng);
    for (double z = 0.0; z <= 4.0; z += 0.1) {
      for (double a = 0.0; a < 2.0 * M_PI; a += 0.5) {
        add(x + 0.1 * std::cos(a), y + 0.1 * std::sin(a), z);
      }
    }
  }
  return map;
}

std::vector<Eigen::Vector3d> makeScan(size_t point_num, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> horizontal(-15.0, 15.0), vertical(-1.5, 1.5);
  std::vector<Eigen::Vector3d> points(point_num);
  for (auto & p : points) {
    p = Eigen::Vector3d(horizontal(rng), horizontal(rng), vertical(rng));
  }
  return points;
}

double measureMs(
  const std::shared_ptr<const PointCloudMap> & map, size_t particle_num, size_t point_num,
  int repetitions, size_t thread_num)
{
  PoseWithCovariance initial;
  initial.pose.position = Eigen::Vector3d(20.0, 20.0, 1.0);
  initial.covariance.diagonal() << 4.0, 4.0, 0.01, 0.001, 0.001, 0.5;
  ParticleFilter filter(1);
  filter.initialize(initial, particle_num);
  std::mt19937 rng(1);
  const PointCloudMeasurementModel model(map, makeScan(point_num, rng), {1.0, 0.2, thread_num});

  double sum = 0.0;  // keeps the result alive
  const auto start = std::chrono::steady_clock::now();
  for (int r = 0; r < repetitions; ++r) {
    sum += model.computeLogLikelihood(filter.particles()).values.sum();
  }
  const auto end = std::chrono::steady_clock::now();
  if (!std::isfinite(sum)) {
    std::printf("unexpected non-finite likelihood\n");
  }
  return std::chrono::duration<double, std::milli>(end - start).count() / repetitions;
}
}  // namespace

int main(int argc, char ** argv)
{
  const int repetitions = argc > 1 ? std::atoi(argv[1]) : 5;
  const auto threads = static_cast<size_t>(argc > 2 ? std::max(0, std::atoi(argv[2])) : 4);
  const PointCloud::Ptr cloud = makeMap();

  auto start = std::chrono::steady_clock::now();
  const auto kdtree = std::make_shared<KdTreeMap>(cloud);
  const double kdtree_build =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  start = std::chrono::steady_clock::now();
  const auto field = std::make_shared<DistanceFieldMap>(*cloud, DistanceFieldMap::Params{0.1, 1.0});
  const double field_build =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  std::printf("map: %zu points, likelihood threads: %zu (0: all)\n", cloud->size(), threads);
  std::printf(
    "build: kdtree %.2f s, distance_field(0.1 m) %.2f s, %.0f MB\n", kdtree_build, field_build,
    static_cast<double>(field->memoryBytes()) / 1e6);
  std::shared_ptr<CudaDistanceFieldMap> cuda_field;
  if (CudaDistanceFieldMap::available()) {
    cuda_field = std::make_shared<CudaDistanceFieldMap>(field);
    measureMs(cuda_field, 100, 100, 1, threads);  // CUDA context and buffer setup
  } else {
    std::printf("CUDA: not available (build with -DAMCL3D_USE_CUDA=ON and a CUDA device)\n");
  }
  std::printf(
    "%10s %8s %12s %12s %12s\n", "particles", "points", "kdtree [ms]", "field [ms]", "cuda [ms]");
  for (const size_t particle_num : {500, 2000, 10000}) {
    for (const size_t point_num : {100, 300, 2000}) {
      // the k-d tree takes seconds for the large cases
      const bool large = particle_num * point_num > 1000000;
      const double kd_ms =
        large ? NAN : measureMs(kdtree, particle_num, point_num, repetitions, threads);
      const double field_ms = measureMs(field, particle_num, point_num, repetitions, threads);
      const double cuda_ms =
        cuda_field ? measureMs(cuda_field, particle_num, point_num, repetitions, threads) : NAN;
      std::printf(
        "%10zu %8zu %12.1f %12.1f %12.2f\n", particle_num, point_num, kd_ms, field_ms, cuda_ms);
    }
  }
  return 0;
}
