#pragma once

#include <memory>

#include "amcl_3d_core/sensor.hpp"
#include "amcl_3d_core/update/measurement_update.hpp"

namespace amcl_3d
{
/// The update of a sensor: TemperedSmcUpdate, AnnealedUpdate or ImportanceUpdate.
/// Throws std::invalid_argument if both annealing and tempered SMC are enabled.
std::unique_ptr<const MeasurementUpdate> createMeasurementUpdate(const SensorParams & sensor);
}  // namespace amcl_3d
