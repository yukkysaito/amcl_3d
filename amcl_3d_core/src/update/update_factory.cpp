#include "amcl_3d_core/update/update_factory.hpp"

#include <stdexcept>

#include "amcl_3d_core/update/annealed_update.hpp"
#include "amcl_3d_core/update/tempered_smc_update.hpp"

namespace amcl_3d
{
std::unique_ptr<const MeasurementUpdate> createMeasurementUpdate(const SensorParams & sensor)
{
  if (sensor.tempered_smc.enabled && sensor.annealing_layer_num > 0) {
    throw std::invalid_argument(
      "sensor '" + sensor.name + "': annealing and tempered SMC are exclusive");
  }
  if (sensor.tempered_smc.enabled) {
    TemperedSmcUpdate::Params params;
    params.max_stages = sensor.tempered_smc.max_stages;
    params.mcmc_steps = sensor.tempered_smc.mcmc_steps;
    params.cess_ratio = sensor.tempered_smc.cess_ratio;
    const size_t threads = sensor.tempered_smc.thread_num;
    std::shared_ptr<const MoveKernel> kernel;
    switch (sensor.tempered_smc.kernel) {
      case SensorParams::TemperedSmc::Kernel::kRandomWalk:
        kernel = std::make_shared<RandomWalkMetropolis>(threads);
        break;
      case SensorParams::TemperedSmc::Kernel::kMala:
        kernel = std::make_shared<LangevinMove>(LangevinMove::Preconditioner::kIsotropic, threads);
        break;
      case SensorParams::TemperedSmc::Kernel::kPreconditionedMala:
        kernel = std::make_shared<LangevinMove>(LangevinMove::Preconditioner::kCluster, threads);
        break;
      case SensorParams::TemperedSmc::Kernel::kGaussNewton:
        kernel =
          std::make_shared<LangevinMove>(LangevinMove::Preconditioner::kGaussNewton, threads);
        break;
    }
    return std::make_unique<TemperedSmcUpdate>(params, kernel);
  }
  if (sensor.annealing_layer_num > 0) {
    return std::make_unique<AnnealedUpdate>(sensor.annealing_layer_num);
  }
  return std::make_unique<ImportanceUpdate>();
}
}  // namespace amcl_3d
