#include <cuda_runtime.h>

#include <stdexcept>
#include <string>

#include "device_distance_field.hpp"

namespace amcl_3d
{
namespace cuda
{
namespace
{
constexpr int kThreadsPerParticle = 128;  // one CUDA block per particle
constexpr int kWarpSize = 32;
constexpr int8_t kNoSite = -128;  // same marker as DistanceFieldMap

void check(cudaError_t error, const char * what)
{
  if (error != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA ") + what + ": " + cudaGetErrorString(error));
  }
}

/// Device buffer that only grows, reused between calls.
template <typename T>
class Buffer
{
public:
  Buffer() = default;
  Buffer(const Buffer &) = delete;
  Buffer & operator=(const Buffer &) = delete;
  ~Buffer() { cudaFree(data_); }

  void reserve(size_t size)
  {
    if (size <= capacity_) {
      return;
    }
    cudaFree(data_);
    data_ = nullptr;
    capacity_ = 0;
    check(cudaMalloc(&data_, size * sizeof(T)), "malloc");
    capacity_ = size;
  }
  T * data() const { return data_; }
  size_t bytes() const { return capacity_ * sizeof(T); }

private:
  T * data_ = nullptr;
  size_t capacity_ = 0;
};

__device__ int floorDiv(int a, int b)  // b > 0
{
  const int q = a / b;
  return (a % b != 0 && a < 0) ? q - 1 : q;
}

/// Residual point - nearest site (DistanceFieldMap::nearestPoint()); false without a site.
__device__ bool siteResidual(
  float x, float y, float z, const FieldLayout & layout, const int8_t * site_offsets,
  const int32_t * block_lookup, float & rx, float & ry, float & rz)
{
  const float sx = floorf(x / layout.resolution);
  const float sy = floorf(y / layout.resolution);
  const float sz = floorf(z / layout.resolution);
  if (!(fabsf(sx) <= 1e9f && fabsf(sy) <= 1e9f && fabsf(sz) <= 1e9f)) {
    return false;
  }
  const int vx = static_cast<int>(sx);
  const int vy = static_cast<int>(sy);
  const int vz = static_cast<int>(sz);
  const int b = layout.block_size;
  const int bx = floorDiv(vx, b);
  const int by = floorDiv(vy, b);
  const int bz = floorDiv(vz, b);
  if (
    bx < 0 || by < 0 || bz < 0 || bx >= layout.block_dims[0] || by >= layout.block_dims[1] ||
    bz >= layout.block_dims[2]) {
    return false;
  }
  const int32_t block =
    block_lookup[(static_cast<size_t>(bz) * layout.block_dims[1] + by) * layout.block_dims[0] + bx];
  if (block < 0) {
    return false;
  }
  const size_t voxel = static_cast<size_t>(block) * b * b * b +
                       (static_cast<size_t>(vz - bz * b) * b + (vy - by * b)) * b + (vx - bx * b);
  const int8_t * offset = site_offsets + voxel * 3;
  if (offset[0] == kNoSite) {
    return false;
  }
  rx = x - ((vx + 0.5f) * layout.resolution + offset[0] * layout.offset_scale);
  ry = y - ((vy + 0.5f) * layout.resolution + offset[1] * layout.offset_scale);
  rz = z - ((vz + 0.5f) * layout.resolution + offset[2] * layout.offset_scale);
  return true;
}

/// Mirrors DistanceFieldMap::squaredDistanceToNearest(), clipped at clip_squared (which is not
/// larger than the squared max_distance of the field).
__device__ float clippedSquaredDistance(
  float x, float y, float z, const FieldLayout & layout, const int8_t * site_offsets,
  const int32_t * block_lookup, float clip_squared)
{
  const float sx = floorf(x / layout.resolution);
  const float sy = floorf(y / layout.resolution);
  const float sz = floorf(z / layout.resolution);
  // also rejects NaN, for which every comparison is false
  if (!(fabsf(sx) <= 1e9f && fabsf(sy) <= 1e9f && fabsf(sz) <= 1e9f)) {
    return clip_squared;
  }
  const int vx = static_cast<int>(sx);
  const int vy = static_cast<int>(sy);
  const int vz = static_cast<int>(sz);
  const int b = layout.block_size;
  const int bx = floorDiv(vx, b);
  const int by = floorDiv(vy, b);
  const int bz = floorDiv(vz, b);
  if (
    bx < 0 || by < 0 || bz < 0 || bx >= layout.block_dims[0] || by >= layout.block_dims[1] ||
    bz >= layout.block_dims[2]) {
    return clip_squared;
  }
  const int32_t block =
    block_lookup[(static_cast<size_t>(bz) * layout.block_dims[1] + by) * layout.block_dims[0] + bx];
  if (block < 0) {
    return clip_squared;
  }
  const size_t voxel = static_cast<size_t>(block) * b * b * b +
                       (static_cast<size_t>(vz - bz * b) * b + (vy - by * b)) * b + (vx - bx * b);
  const int8_t * offset = site_offsets + voxel * 3;
  if (offset[0] == kNoSite) {
    return clip_squared;
  }
  const float dx = (vx + 0.5f) * layout.resolution + offset[0] * layout.offset_scale - x;
  const float dy = (vy + 0.5f) * layout.resolution + offset[1] * layout.offset_scale - y;
  const float dz = (vz + 0.5f) * layout.resolution + offset[2] * layout.offset_scale - z;
  return fminf(dx * dx + dy * dy + dz * dz, clip_squared);
}

__global__ void sumClippedSquaredDistancesKernel(
  FieldLayout layout, const int8_t * site_offsets, const int32_t * block_lookup,
  const Transform * transforms, const float3 * points, int point_num, float clip_squared,
  double * sums)
{
  __shared__ Transform transform;
  __shared__ double warp_sums[kThreadsPerParticle / kWarpSize];
  const int particle = blockIdx.x;
  constexpr int kTransformFloats = sizeof(Transform) / sizeof(float);
  if (threadIdx.x < kTransformFloats) {
    reinterpret_cast<float *>(&transform)[threadIdx.x] =
      reinterpret_cast<const float *>(&transforms[particle])[threadIdx.x];
  }
  __syncthreads();

  const float * r = transform.rotation;
  const float * t = transform.translation;
  float sum = 0.0f;  // a few points per thread: single precision is enough here
  for (int k = threadIdx.x; k < point_num; k += blockDim.x) {
    const float3 p = points[k];
    sum += clippedSquaredDistance(
      r[0] * p.x + r[1] * p.y + r[2] * p.z + t[0], r[3] * p.x + r[4] * p.y + r[5] * p.z + t[1],
      r[6] * p.x + r[7] * p.y + r[8] * p.z + t[2], layout, site_offsets, block_lookup,
      clip_squared);
  }

  // block reduction in double: warp shuffles, then one value per warp
  double value = sum;
  for (int offset = kWarpSize / 2; offset > 0; offset /= 2) {
    value += __shfl_down_sync(0xffffffff, value, offset);
  }
  if (threadIdx.x % kWarpSize == 0) {
    warp_sums[threadIdx.x / kWarpSize] = value;
  }
  __syncthreads();
  if (threadIdx.x == 0) {
    double total = 0.0;
    for (int w = 0; w < kThreadsPerParticle / kWarpSize; ++w) {
      total += warp_sums[w];
    }
    sums[particle] = total;
  }
}
constexpr int kTerms = DeviceDistanceField::kDerivativeTerms;

__global__ void sumClippedSquaredDistanceTermsKernel(
  FieldLayout layout, const int8_t * site_offsets, const int32_t * block_lookup,
  const Transform * transforms, const float3 * points, int point_num, float clip_squared,
  double * terms)
{
  __shared__ Transform transform;
  __shared__ double warp_sums[kThreadsPerParticle / kWarpSize][kTerms];
  const int particle = blockIdx.x;
  constexpr int kTransformFloats = sizeof(Transform) / sizeof(float);
  if (threadIdx.x < kTransformFloats) {
    reinterpret_cast<float *>(&transform)[threadIdx.x] =
      reinterpret_cast<const float *>(&transforms[particle])[threadIdx.x];
  }
  __syncthreads();

  const float * r = transform.rotation;
  const float * t = transform.translation;
  float acc[kTerms];
  for (int j = 0; j < kTerms; ++j) {
    acc[j] = 0.0f;
  }
  for (int k = threadIdx.x; k < point_num; k += blockDim.x) {
    const float3 p = points[k];
    const float x = r[0] * p.x + r[1] * p.y + r[2] * p.z + t[0];
    const float y = r[3] * p.x + r[4] * p.y + r[5] * p.z + t[1];
    const float z = r[6] * p.x + r[7] * p.y + r[8] * p.z + t[2];
    float rx, ry, rz;
    const bool found = siteResidual(x, y, z, layout, site_offsets, block_lookup, rx, ry, rz);
    const float squared = found ? rx * rx + ry * ry + rz * rz : clip_squared;
    if (!found || squared >= clip_squared) {
      acc[0] += clip_squared;
      continue;
    }
    acc[0] += squared;
    // u = R^T residual
    const float ux = r[0] * rx + r[3] * ry + r[6] * rz;
    const float uy = r[1] * rx + r[4] * ry + r[7] * rz;
    const float uz = r[2] * rx + r[5] * ry + r[8] * rz;
    acc[1] += ux;
    acc[2] += uy;
    acc[3] += uz;
    acc[4] += p.y * uz - p.z * uy;
    acc[5] += p.z * ux - p.x * uz;
    acc[6] += p.x * uy - p.y * ux;
    acc[7] += 1.0f;
    acc[8] += p.x;
    acc[9] += p.y;
    acc[10] += p.z;
    acc[11] += p.x * p.x;
    acc[12] += p.x * p.y;
    acc[13] += p.x * p.z;
    acc[14] += p.y * p.y;
    acc[15] += p.y * p.z;
    acc[16] += p.z * p.z;
  }
  for (int j = 0; j < kTerms; ++j) {
    double value = acc[j];
    for (int offset = kWarpSize / 2; offset > 0; offset /= 2) {
      value += __shfl_down_sync(0xffffffff, value, offset);
    }
    if (threadIdx.x % kWarpSize == 0) {
      warp_sums[threadIdx.x / kWarpSize][j] = value;
    }
  }
  __syncthreads();
  if (threadIdx.x < kTerms) {
    double total = 0.0;
    for (int w = 0; w < kThreadsPerParticle / kWarpSize; ++w) {
      total += warp_sums[w][threadIdx.x];
    }
    terms[static_cast<size_t>(particle) * kTerms + threadIdx.x] = total;
  }
}
}  // namespace

struct DeviceDistanceField::Impl
{
  int device = 0;
  FieldLayout layout{};
  cudaStream_t stream = nullptr;
  Buffer<int8_t> site_offsets;
  Buffer<int32_t> block_lookup;
  Buffer<Transform> transforms;
  Buffer<float3> points;
  Buffer<double> sums;
  Buffer<double> terms;

  ~Impl()
  {
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
    }
  }
};

int deviceCount()
{
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess ? count : 0;
}

DeviceDistanceField::DeviceDistanceField(
  int device, const FieldLayout & layout, const std::vector<int8_t> & site_offsets,
  const std::vector<int32_t> & block_lookup)
: impl_(std::make_unique<Impl>())
{
  impl_->device = device;
  impl_->layout = layout;
  check(cudaSetDevice(device), "set device");
  check(cudaStreamCreateWithFlags(&impl_->stream, cudaStreamNonBlocking), "create stream");
  impl_->site_offsets.reserve(site_offsets.size());
  impl_->block_lookup.reserve(block_lookup.size());
  check(
    cudaMemcpy(
      impl_->site_offsets.data(), site_offsets.data(), site_offsets.size(), cudaMemcpyHostToDevice),
    "upload site offsets");
  check(
    cudaMemcpy(
      impl_->block_lookup.data(), block_lookup.data(), block_lookup.size() * sizeof(int32_t),
      cudaMemcpyHostToDevice),
    "upload block lookup");
}

DeviceDistanceField::~DeviceDistanceField() = default;

void DeviceDistanceField::sumClippedSquaredDistances(
  const std::vector<Transform> & transforms, const std::vector<float> & points, float clip_squared,
  std::vector<double> & sums)
{
  const size_t particle_num = transforms.size();
  const size_t point_num = points.size() / 3;
  sums.assign(particle_num, 0.0);
  if (particle_num == 0 || point_num == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  Impl & d = *impl_;
  check(cudaSetDevice(d.device), "set device");
  d.transforms.reserve(particle_num);
  d.points.reserve(point_num);
  d.sums.reserve(particle_num);
  check(
    cudaMemcpyAsync(
      d.transforms.data(), transforms.data(), particle_num * sizeof(Transform),
      cudaMemcpyHostToDevice, d.stream),
    "upload transforms");
  check(
    cudaMemcpyAsync(
      d.points.data(), points.data(), points.size() * sizeof(float), cudaMemcpyHostToDevice,
      d.stream),
    "upload points");
  sumClippedSquaredDistancesKernel<<<
    static_cast<unsigned int>(particle_num), kThreadsPerParticle, 0, d.stream>>>(
    d.layout, d.site_offsets.data(), d.block_lookup.data(), d.transforms.data(), d.points.data(),
    static_cast<int>(point_num), clip_squared, d.sums.data());
  check(cudaGetLastError(), "launch");
  check(
    cudaMemcpyAsync(
      sums.data(), d.sums.data(), particle_num * sizeof(double), cudaMemcpyDeviceToHost, d.stream),
    "download sums");
  check(cudaStreamSynchronize(d.stream), "synchronize");
}

void DeviceDistanceField::sumClippedSquaredDistanceTerms(
  const std::vector<Transform> & transforms, const std::vector<float> & points, float clip_squared,
  std::vector<double> & terms)
{
  const size_t particle_num = transforms.size();
  const size_t point_num = points.size() / 3;
  terms.assign(particle_num * kTerms, 0.0);
  if (particle_num == 0) {
    return;
  }
  if (point_num == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  Impl & d = *impl_;
  check(cudaSetDevice(d.device), "set device");
  d.transforms.reserve(particle_num);
  d.points.reserve(point_num);
  d.terms.reserve(particle_num * kTerms);
  check(
    cudaMemcpyAsync(
      d.transforms.data(), transforms.data(), particle_num * sizeof(Transform),
      cudaMemcpyHostToDevice, d.stream),
    "upload transforms");
  check(
    cudaMemcpyAsync(
      d.points.data(), points.data(), points.size() * sizeof(float), cudaMemcpyHostToDevice,
      d.stream),
    "upload points");
  sumClippedSquaredDistanceTermsKernel<<<
    static_cast<unsigned int>(particle_num), kThreadsPerParticle, 0, d.stream>>>(
    d.layout, d.site_offsets.data(), d.block_lookup.data(), d.transforms.data(), d.points.data(),
    static_cast<int>(point_num), clip_squared, d.terms.data());
  check(cudaGetLastError(), "launch");
  check(
    cudaMemcpyAsync(
      terms.data(), d.terms.data(), particle_num * kTerms * sizeof(double), cudaMemcpyDeviceToHost,
      d.stream),
    "download terms");
  check(cudaStreamSynchronize(d.stream), "synchronize");
}

size_t DeviceDistanceField::memoryBytes() const
{
  const Impl & d = *impl_;
  return d.site_offsets.bytes() + d.block_lookup.bytes() + d.transforms.bytes() + d.points.bytes() +
         d.sums.bytes() + d.terms.bytes();
}

}  // namespace cuda
}  // namespace amcl_3d
