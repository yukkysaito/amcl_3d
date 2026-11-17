// Stands in for device_distance_field.cu when amcl_3d_core is built without CUDA, so that
// CudaDistanceFieldMap reports the missing backend at run time instead of every user needing
// #ifdefs.
#include <stdexcept>

#include "device_distance_field.hpp"

namespace amcl_3d
{
namespace cuda
{
struct DeviceDistanceField::Impl
{
};

int deviceCount() { return 0; }

DeviceDistanceField::DeviceDistanceField(
  int, const FieldLayout &, const std::vector<int8_t> &, const std::vector<int32_t> &)
{
  throw std::runtime_error("amcl_3d_core was built without CUDA (-DAMCL3D_USE_CUDA=ON)");
}

DeviceDistanceField::~DeviceDistanceField() = default;

void DeviceDistanceField::sumClippedSquaredDistances(
  const std::vector<Transform> &, const std::vector<float> &, float, std::vector<double> &)
{
  throw std::runtime_error("amcl_3d_core was built without CUDA (-DAMCL3D_USE_CUDA=ON)");
}

size_t DeviceDistanceField::memoryBytes() const { return 0; }

}  // namespace cuda
}  // namespace amcl_3d
