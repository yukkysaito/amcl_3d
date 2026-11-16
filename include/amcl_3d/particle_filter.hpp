#pragma once
#include <memory>
#include "amcl_3d/type.hpp"
#include "amcl_3d/time.hpp"
#include "amcl_3d/resample_noise/resample_noise_interface.hpp"
#include "amcl_3d/resample_noise/normal_distribution.hpp"
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"
#include "amcl_3d/measurement_model/measurement_model_interface.hpp"
#include "amcl_3d/xorshift128.hpp"
#include <random>
#include <limits>

namespace amcl_3d
{
class ParticleFilter
{
public:
  struct NoiseGenerators
  {
    NoiseGenerators(std::shared_ptr<ResampleNoiseInterface> x_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> y_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> z_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> roll_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> pitch_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> yaw_noise_gen) : x(x_noise_gen), y(y_noise_gen), z(z_noise_gen), roll(roll_noise_gen), pitch(pitch_noise_gen), yaw(yaw_noise_gen) {}
    std::shared_ptr<ResampleNoiseInterface> x;
    std::shared_ptr<ResampleNoiseInterface> y;
    std::shared_ptr<ResampleNoiseInterface> z;
    std::shared_ptr<ResampleNoiseInterface> roll;
    std::shared_ptr<ResampleNoiseInterface> pitch;
    std::shared_ptr<ResampleNoiseInterface> yaw;
  };

public:
  ParticleFilter();
  explicit ParticleFilter(uint64_t seed);

  virtual ~ParticleFilter(){};
  bool init(const Position &position, const Quat &quat, const NoiseGenerators &noise_gen, const size_t particle_num,
            const Time &time);
  bool init(const Position &position, const Quat &quat, const NoiseGenerators &noise_gen, const size_t particle_num); // time = now
  bool resample(const KLDSamplingParam &param, const double random_particle_ratio,
                const NoiseGenerators &random_particle_noise_gen); // kld resample and random noised resample
  bool resample(const size_t particle_num,
                const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen); // random noised resample
  bool resample(const size_t particle_num);                                                            // normal resample
  bool predict(std::shared_ptr<PredictionModelInterface> model, const Time &time);
  // update_time == true : advance internal clock. A time older than the internal clock is rejected.
  // update_time == false: predict a copy of particles (e.g. to a delayed measurement stamp), dt may be negative.
  bool predict(std::shared_ptr<Particles> particles_ptr,
               std::shared_ptr<PredictionModelInterface> model,
               const Time &time,
               const bool update_time = true);
  bool measure(std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state);
  bool measure(std::shared_ptr<const Particles> measurement_point_particles_ptr,
               std::shared_ptr<MeasurementModelInterface> model,
               MeasurementState &measurement_state);
  bool getParticles(std::shared_ptr<const Particles> &particles_ptr);
  void normalizeWeight();
  State getMMSE(); // minimum mean square error
  State getMAP();  // maximum a posteriori
  size_t getParticleNum();
  double getESS(const bool normalized = false); // effective sample size
  Time getLastPredictionTime() const { return last_prediction_time_; }

  // inverse of the standard normal CDF (quantile function), p in (0, 1)
  static double getNormalQuantile(const double p);

private:
  // draw `particle_num` indices by low variance (systematic) sampling with a random offset
  std::vector<size_t> drawSystematicIndices(const size_t particle_num);
  State addNoise(const State &state, const NoiseGenerators &noise_gen);
  bool resample(std::shared_ptr<Particles> new_particles_ptr, const size_t particle_num,
                const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen);
  void normalizeWeight(std::shared_ptr<Particles> particles_ptr);
  XorShift128 rand_;
  std::shared_ptr<Particles> particles_ptr_;
  Time last_prediction_time_;
};
} // namespace amcl_3d
