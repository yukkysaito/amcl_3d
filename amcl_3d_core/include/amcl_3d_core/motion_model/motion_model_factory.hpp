#pragma once

#include <memory>
#include <string>

#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
#include "amcl_3d_core/motion_model/vehicle_motion_model.hpp"

namespace amcl_3d
{
/// Selects the motion model built by the Localizer. Custom models can still be installed with
/// Localizer::setMotionModel().
struct MotionModelParams
{
  enum class Type
  {
    kTwist,    // generic 6-DoF body twist with isotropic noise (robots, handheld sensors)
    kVehicle,  // car-like: forward speed and yaw rate, anisotropic noise
  };

  Type type = Type::kTwist;
  TwistMotionModel::Params twist;
  VehicleMotionModel::Params vehicle;
};

/// Parses "twist" or "vehicle". Throws std::invalid_argument otherwise.
MotionModelParams::Type toMotionModelType(const std::string & name);

std::unique_ptr<MotionModel> createMotionModel(const MotionModelParams & params);

}  // namespace amcl_3d
