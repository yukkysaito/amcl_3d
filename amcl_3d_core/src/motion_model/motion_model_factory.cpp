#include "amcl_3d_core/motion_model/motion_model_factory.hpp"

#include <stdexcept>

namespace amcl_3d
{
MotionModelParams::Type toMotionModelType(const std::string & name)
{
  if (name == "twist") {
    return MotionModelParams::Type::kTwist;
  }
  if (name == "vehicle") {
    return MotionModelParams::Type::kVehicle;
  }
  throw std::invalid_argument("motion model type must be 'twist' or 'vehicle': " + name);
}

std::unique_ptr<MotionModel> createMotionModel(const MotionModelParams & params)
{
  switch (params.type) {
    case MotionModelParams::Type::kTwist:
      return std::make_unique<TwistMotionModel>(params.twist);
    case MotionModelParams::Type::kVehicle:
      return std::make_unique<VehicleMotionModel>(params.vehicle);
  }
  throw std::invalid_argument("unknown motion model type");
}

}  // namespace amcl_3d
