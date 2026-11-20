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
  const int gx = bx - layout.block_min[0];
  const int gy = by - layout.block_min[1];
  const int gz = bz - layout.block_min[2];
  if (
    gx < 0 || gy < 0 || gz < 0 || gx >= layout.block_dims[0] || gy >= layout.block_dims[1] ||
    gz >= layout.block_dims[2]) {
    return clip_squared;
  }
  const int32_t block =
    block_lookup[(static_cast<size_t>(gz) * layout.block_dims[1] + gy) * layout.block_dims[0] + gx];
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

size_t DeviceDistanceField::memoryBytes() const
{
  const Impl & d = *impl_;
  return d.site_offsets.bytes() + d.block_lookup.bytes() + d.transforms.bytes() + d.points.bytes() +
         d.sums.bytes();
}

}  // namespace cuda
}  // namespace amcl_3d
