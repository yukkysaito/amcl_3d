#include "amcl_3d_core/update/update_factory.hpp"

#include <stdexcept>

#include "amcl_3d_core/update/tempered_smc_update.hpp"

namespace amcl_3d
{
std::unique_ptr<const MeasurementUpdate> createMeasurementUpdate(const SensorParams & sensor)
{
  if (sensor.tempered_smc.enabled) {
    TemperedSmcUpdate::Params params;
    params.max_stages = sensor.tempered_smc.max_stages;
    params.mcmc_steps = sensor.tempered_smc.mcmc_steps;
    params.cess_ratio = sensor.tempered_smc.cess_ratio;
    params.coarse_to_fine = sensor.tempered_smc.coarse_to_fine;
    const size_t threads = sensor.tempered_smc.thread_num;
    std::shared_ptr<const MoveKernel> kernel;
    switch (sensor.tempered_smc.kernel) {
      case SensorParams::TemperedSmc::Kernel::kRandomWalk:
        kernel = std::make_shared<RandomWalkMetropolis>(threads);
        break;
      case SensorParams::TemperedSmc::Kernel::kGaussNewton:
        kernel = std::make_shared<GaussNewtonMove>(threads);
        break;
      case SensorParams::TemperedSmc::Kernel::kVerticalGaussNewton:
        kernel = std::make_shared<GaussNewtonMove>(threads, true);
        break;
    }
    return std::make_unique<TemperedSmcUpdate>(params, kernel);
  }
  return std::make_unique<ImportanceUpdate>();
}
}  // namespace amcl_3d
