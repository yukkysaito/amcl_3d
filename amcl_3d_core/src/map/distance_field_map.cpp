#include "amcl_3d_core/map/distance_field_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace amcl_3d
{
namespace
{
constexpr int kMinBlockSize = 16;
constexpr double kInfinity = 1e20;  // finite "infinity" keeps the envelope arithmetic NaN free

int floorDiv(int a, int b)
{
  const int q = a / b;
  return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

Eigen::Vector3i floorDiv(const Eigen::Vector3i & v, int b)
{
  return Eigen::Vector3i(floorDiv(v.x(), b), floorDiv(v.y(), b), floorDiv(v.z(), b));
}

/// 1D squared Euclidean distance transform of the sampled function f with the given stride
/// (Felzenszwalb & Huttenlocher, "Distance Transforms of Sampled Functions", 2012):
///   d(p) = min_q ((p - q)^2 + f(q))
/// computed in place in O(n) as the lower envelope of parabolas. `sites` (same layout as `data`)
/// is carried along with the minimizer, which turns the separable transform into a feature
/// transform: each cell ends up with the site of its nearest seed.
void distanceTransform1d(
  double * data, Eigen::Vector3f * sites, int n, int stride, std::vector<double> & f,
  std::vector<Eigen::Vector3f> & f_sites, std::vector<int> & v, std::vector<double> & z)
{
  for (int i = 0; i < n; ++i) {
    f[i] = data[i * stride];
    f_sites[i] = sites[i * stride];
  }
  // parabola rooted at q intersects the one rooted at r at s
  auto intersection = [&f](int q, int r) {
    return ((f[q] + q * q) - (f[r] + r * r)) / (2.0 * (q - r));
  };
  int k = 0;  // index of the rightmost parabola in the lower envelope
  v[0] = 0;
  z[0] = -kInfinity;
  z[1] = kInfinity;
  for (int q = 1; q < n; ++q) {
    double s = intersection(q, v[k]);
    while (s <= z[k]) {
      --k;
      s = intersection(q, v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInfinity;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    data[q * stride] = (q - v[k]) * (q - v[k]) + f[v[k]];
    sites[q * stride] = f_sites[v[k]];
  }
}

struct Centroid
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;
};

/// Occupied voxel with the centroid of the map points inside it.
struct Seed
{
  Eigen::Vector3i voxel;
  Eigen::Vector3f centroid;
};
}  // namespace

size_t DistanceFieldMap::KeyHash::operator()(const Eigen::Vector3i & key) const
{
  // large primes (Teschner et al., 2003)
  return static_cast<size_t>(key.x()) * 73856093u ^ static_cast<size_t>(key.y()) * 19349663u ^
         static_cast<size_t>(key.z()) * 83492791u;
}

DistanceFieldMap::DistanceFieldMap(const PointCloud & cloud, const Params & params)
: params_(params)
{
  if (cloud.empty()) {
    throw std::invalid_argument("DistanceFieldMap: the map point cloud is empty");
  }
  if (params_.resolution <= 0.0 || params_.max_distance <= 0.0) {
    throw std::invalid_argument("DistanceFieldMap: resolution and max_distance must be positive");
  }
  padding_ = static_cast<int>(std::ceil(params_.max_distance / params_.resolution));
  block_size_ = std::max(kMinBlockSize, padding_);  // so that only adjacent blocks interact
  offset_scale_ = params_.max_distance / std::numeric_limits<int8_t>::max();
  build(cloud);
}

void DistanceFieldMap::build(const PointCloud & cloud)
{
  const int b = block_size_;
  const int p = padding_;
  const double r = params_.resolution;

  // 1. occupied voxels with the centroid of their points, bucketed by block
  std::unordered_map<Eigen::Vector3i, Centroid, KeyHash> occupied;
  for (const auto & point : cloud) {
    const Eigen::Vector3d position(point.x, point.y, point.z);
    if (position.allFinite()) {
      Centroid & centroid = occupied[(position / r).array().floor().cast<int>()];
      centroid.sum += position;
      ++centroid.count;
    }
  }
  std::unordered_map<Eigen::Vector3i, std::vector<Seed>, KeyHash> seeds_by_block;
  std::unordered_set<Eigen::Vector3i, KeyHash> active_blocks;  // blocks within max_distance
  for (const auto & [voxel, centroid] : occupied) {
    seeds_by_block[floorDiv(voxel, b)].push_back(
      Seed{voxel, (centroid.sum / centroid.count).cast<float>()});
    const Eigen::Vector3i min_block = floorDiv(voxel.array() - p, b);
    const Eigen::Vector3i max_block = floorDiv(voxel.array() + p, b);
    for (int z = min_block.z(); z <= max_block.z(); ++z) {
      for (int y = min_block.y(); y <= max_block.y(); ++y) {
        for (int x = min_block.x(); x <= max_block.x(); ++x) {
          active_blocks.insert(Eigen::Vector3i(x, y, z));
        }
      }
    }
  }

  // 2. per block, find the nearest seed of every voxel with a feature transform on a grid padded
  //    by max_distance, then store the seed's centroid relative to the voxel center.
  const std::vector<Eigen::Vector3i> blocks(active_blocks.begin(), active_blocks.end());
  const int block_voxel_num = b * b * b;
  site_offsets_.assign(blocks.size() * static_cast<size_t>(block_voxel_num) * 3, kNoSite);
  const int n = b + 2 * p;  // padded edge length
  const Eigen::Vector3f no_site =
    Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());

#pragma omp parallel for schedule(dynamic)
  for (int64_t block_i = 0; block_i < static_cast<int64_t>(blocks.size()); ++block_i) {
    const Eigen::Vector3i & block = blocks[static_cast<size_t>(block_i)];
    const Eigen::Vector3i origin = block * b - Eigen::Vector3i::Constant(p);
    const size_t grid_size = static_cast<size_t>(n) * n * n;
    std::vector<double> grid(grid_size, kInfinity);
    std::vector<Eigen::Vector3f> sites(grid_size, no_site);
    auto index = [n](int x, int y, int z) { return (static_cast<size_t>(z) * n + y) * n + x; };

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const auto it = seeds_by_block.find(block + Eigen::Vector3i(dx, dy, dz));
          if (it == seeds_by_block.end()) {
            continue;
          }
          for (const auto & seed : it->second) {
            const Eigen::Vector3i local = seed.voxel - origin;
            if ((local.array() >= 0).all() && (local.array() < n).all()) {
              const size_t i = index(local.x(), local.y(), local.z());
              grid[i] = 0.0;
              sites[i] = seed.centroid;
            }
          }
        }
      }
    }

    // separable: x, then y, then z lines
    std::vector<double> f(n);
    std::vector<Eigen::Vector3f> f_sites(n);
    std::vector<int> v(n);
    std::vector<double> z(n + 1);
    auto transform_lines = [&](int stride, auto line_start) {
      for (int outer = 0; outer < n; ++outer) {
        for (int inner = 0; inner < n; ++inner) {
          const size_t start = line_start(inner, outer);
          distanceTransform1d(&grid[start], &sites[start], n, stride, f, f_sites, v, z);
        }
      }
    };
    transform_lines(1, [&](int j, int k) { return index(0, j, k); });      // along x
    transform_lines(n, [&](int i, int k) { return index(i, 0, k); });      // along y
    transform_lines(n * n, [&](int i, int j) { return index(i, j, 0); });  // along z

    int8_t * out = &site_offsets_[static_cast<size_t>(block_i) * block_voxel_num * 3];
    for (int lz = 0; lz < b; ++lz) {
      for (int ly = 0; ly < b; ++ly) {
        for (int lx = 0; lx < b; ++lx) {
          const Eigen::Vector3f & site = sites[index(lx + p, ly + p, lz + p)];
          const Eigen::Vector3d center =
            ((origin + Eigen::Vector3i(lx + p, ly + p, lz + p)).cast<double>().array() + 0.5) * r;
          const Eigen::Vector3d offset = site.cast<double>() - center;
          const bool in_range = site.allFinite() && offset.norm() <= params_.max_distance;
          for (int axis = 0; axis < 3; ++axis) {
            *out++ =
              in_range ? static_cast<int8_t>(std::lround(offset[axis] / offset_scale_)) : kNoSite;
          }
        }
      }
    }
  }

  block_index_.reserve(blocks.size());
  for (size_t i = 0; i < blocks.size(); ++i) {
    block_index_.emplace(blocks[i], static_cast<uint32_t>(i));
  }
}

double DistanceFieldMap::squaredDistanceToNearest(const Eigen::Vector3d & point) const
{
  const double max_squared_distance = params_.max_distance * params_.max_distance;
  const auto site = nearestPoint(point);
  return site ? std::min((point - *site).squaredNorm(), max_squared_distance)
              : max_squared_distance;
}

std::optional<Eigen::Vector3d> DistanceFieldMap::nearestPoint(const Eigen::Vector3d & point) const
{
  const Eigen::Vector3d scaled = (point / params_.resolution).array().floor();
  if (!scaled.allFinite() || scaled.cwiseAbs().maxCoeff() > 1e9) {
    return std::nullopt;
  }
  const Eigen::Vector3i voxel = scaled.cast<int>();
  const Eigen::Vector3i block = floorDiv(voxel, block_size_);
  const auto it = block_index_.find(block);
  if (it == block_index_.end()) {
    return std::nullopt;
  }
  const Eigen::Vector3i local = voxel - block * block_size_;
  const size_t voxel_index =
    static_cast<size_t>(it->second) * block_size_ * block_size_ * block_size_ +
    (static_cast<size_t>(local.z()) * block_size_ + local.y()) * block_size_ + local.x();
  const int8_t * offset = &site_offsets_[voxel_index * 3];
  if (offset[0] == kNoSite) {
    return std::nullopt;
  }
  return Eigen::Vector3d(
    (voxel.cast<double>().array() + 0.5) * params_.resolution +
    Eigen::Vector3d(offset[0], offset[1], offset[2]).array() * offset_scale_);
}

std::string DistanceFieldMap::description() const
{
  char text[128];
  std::snprintf(
    text, sizeof(text), "distance field: %.2f m voxels, %zu blocks, %.1f MB", params_.resolution,
    blockNum(), static_cast<double>(memoryBytes()) / 1e6);
  return text;
}

std::vector<Eigen::Vector3i> DistanceFieldMap::blockCoordinates() const
{
  std::vector<Eigen::Vector3i> blocks(block_index_.size());
  for (const auto & [block, index] : block_index_) {
    blocks[index] = block;
  }
  return blocks;
}

}  // namespace amcl_3d
