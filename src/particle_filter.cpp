#include "amcl_3d/particle_filter.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <Eigen/Eigenvalues>

namespace amcl_3d
{
namespace
{
struct BinHash
{
    size_t operator()(const Eigen::Vector3i &bin) const
    {
        size_t h = std::hash<int>()(bin.x());
        h ^= std::hash<int>()(bin.y()) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(bin.z()) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

Eigen::Vector3i toBin(const Position &position, const KLDSamplingParam &param)
{
    return Eigen::Vector3i((int)std::floor(position.x() / param.x_bin_width),
                           (int)std::floor(position.y() / param.y_bin_width),
                           (int)std::floor(position.z() / param.z_bin_width));
}

// number of particles required by KLD sampling (Fox, 2003) for k occupied bins
double getKLDBound(const size_t k, const double epsilon, const double z)
{
    if (k <= 1)
        return 1.0;
    const double km1 = (double)(k - 1);
    const double a = 2.0 / (9.0 * km1);
    const double chi = 1.0 - a + std::sqrt(a) * z;
    return km1 / (2.0 * epsilon) * chi * chi * chi;
}
} // namespace

ParticleFilter::ParticleFilter() : particles_ptr_(std::make_shared<Particles>()) {}
ParticleFilter::ParticleFilter(uint64_t seed) : rand_(seed), particles_ptr_(std::make_shared<Particles>()) {}

State ParticleFilter::addNoise(const State &state, const NoiseGenerators &noise_gen)
{
    State noised_state;
    noised_state.position.x() = state.position.x() + noise_gen.x->getNoise(rand_);
    noised_state.position.y() = state.position.y() + noise_gen.y->getNoise(rand_);
    noised_state.position.z() = state.position.z() + noise_gen.z->getNoise(rand_);
    const Quat noise_quat(Eigen::AngleAxisd(noise_gen.roll->getNoise(rand_), Eigen::Vector3d::UnitX())    // roll
                          * Eigen::AngleAxisd(noise_gen.pitch->getNoise(rand_), Eigen::Vector3d::UnitY()) // pitch
                          * Eigen::AngleAxisd(noise_gen.yaw->getNoise(rand_), Eigen::Vector3d::UnitZ())); // yaw
    noised_state.quat = (noise_quat * state.quat).normalized();
    noised_state.weight = 1.0;
    return noised_state;
}

bool ParticleFilter::init(const Position &position, const Quat &quat, const NoiseGenerators &noise_gen,
                          const size_t particle_num)
{
    return init(position, quat, noise_gen, particle_num, Time::getTimeNow());
}

bool ParticleFilter::init(const Position &position, const Quat &quat, const NoiseGenerators &noise_gen,
                          const size_t particle_num, const Time &time)
{
    if (particle_num == 0)
        return false;

    State mean_state;
    mean_state.position = position;
    mean_state.quat = quat.normalized();
    mean_state.weight = 1.0;

    particles_ptr_ = std::make_shared<Particles>();
    particles_ptr_->reserve(particle_num);
    for (size_t i = 0; i < particle_num; ++i)
    {
        particles_ptr_->push_back(addNoise(mean_state, noise_gen));
    }
    normalizeWeight(particles_ptr_);

    last_prediction_time_ = time;
    return true;
}

std::vector<size_t> ParticleFilter::drawSystematicIndices(const size_t particle_num)
{
    std::vector<size_t> indices;
    indices.reserve(particle_num);
    if (particles_ptr_->empty() || particle_num == 0)
        return indices;

    std::vector<double> cumulative_weight;
    cumulative_weight.reserve(particles_ptr_->size());
    double weight_sum = 0.0;
    for (const auto &particle : *particles_ptr_)
    {
        weight_sum += particle.weight;
        cumulative_weight.push_back(weight_sum);
    }
    if (!(weight_sum > 0.0) || !std::isfinite(weight_sum))
    {
        // degenerated weights: treat as uniform
        for (size_t i = 0; i < cumulative_weight.size(); ++i)
            cumulative_weight.at(i) = (double)(i + 1);
        weight_sum = (double)cumulative_weight.size();
    }

    const double step = weight_sum / (double)particle_num;
    std::uniform_real_distribution<double> offset_dist(0.0, step);
    const double offset = offset_dist(rand_);
    size_t idx = 0;
    const size_t last_idx = particles_ptr_->size() - 1;
    for (size_t i = 0; i < particle_num; ++i)
    {
        const double target = offset + (double)i * step;
        while (idx < last_idx && cumulative_weight.at(idx) <= target)
            ++idx;
        indices.push_back(idx);
    }
    return indices;
}

bool ParticleFilter::resample(const size_t particle_num)
{
    std::shared_ptr<ResampleNoiseInterface> no_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    ParticleFilter::NoiseGenerators
        noise_gens(no_noise_ptr, no_noise_ptr, no_noise_ptr, no_noise_ptr, no_noise_ptr, no_noise_ptr);
    return resample(particle_num, 0.0, noise_gens);
}

bool ParticleFilter::resample(const size_t particle_num,
                              const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen)
{
    std::shared_ptr<Particles> new_particles_ptr = std::make_shared<Particles>();
    if (!resample(new_particles_ptr, particle_num, random_particle_ratio, random_particle_noise_gen))
        return false;
    particles_ptr_ = new_particles_ptr;
    normalizeWeight(particles_ptr_);
    return true;
}

bool ParticleFilter::resample(const KLDSamplingParam &param, const double random_particle_ratio,
                              const NoiseGenerators &random_particle_noise_gen)
{
    if (particles_ptr_->empty() || param.max_particle_num == 0 || !(param.epsilon > 0.0))
        return false;

    std::vector<double> cumulative_weight;
    cumulative_weight.reserve(particles_ptr_->size());
    double weight_sum = 0.0;
    for (const auto &particle : *particles_ptr_)
    {
        weight_sum += particle.weight;
        cumulative_weight.push_back(weight_sum);
    }
    if (!(weight_sum > 0.0) || !std::isfinite(weight_sum))
    {
        for (size_t i = 0; i < cumulative_weight.size(); ++i)
            cumulative_weight.at(i) = (double)(i + 1);
        weight_sum = (double)cumulative_weight.size();
    }

    // upper standard normal quantile z_{1-delta}
    const double delta = std::min(std::max(param.delta, 1e-9), 1.0 - 1e-9);
    const double z = getNormalQuantile(1.0 - delta);
    const double ratio = std::min(std::max(random_particle_ratio, 0.0), 1.0);
    const size_t min_particle_num = std::min(std::max<size_t>(param.min_particle_num, 1), param.max_particle_num);

    std::uniform_real_distribution<double> weight_dist(0.0, weight_sum);
    std::uniform_real_distribution<double> ratio_dist(0.0, 1.0);
    std::unordered_set<Eigen::Vector3i, BinHash> bins;
    std::shared_ptr<Particles> new_particles_ptr = std::make_shared<Particles>();
    new_particles_ptr->reserve(param.max_particle_num);

    while (new_particles_ptr->size() < param.max_particle_num)
    {
        // draw a particle proportional to its weight
        const double u = weight_dist(rand_);
        size_t idx = std::upper_bound(cumulative_weight.begin(), cumulative_weight.end(), u) - cumulative_weight.begin();
        idx = std::min(idx, particles_ptr_->size() - 1);

        State state;
        if (ratio_dist(rand_) < ratio)
        {
            state = addNoise(particles_ptr_->at(idx), random_particle_noise_gen);
        }
        else
        {
            state = particles_ptr_->at(idx);
            state.weight = 1.0;
        }
        new_particles_ptr->push_back(state);
        bins.insert(toBin(state.position, param));

        const double kld_bound = getKLDBound(bins.size(), param.epsilon, z);
        if (min_particle_num <= new_particles_ptr->size() && kld_bound <= (double)new_particles_ptr->size())
            break;
    }

    particles_ptr_ = new_particles_ptr;
    normalizeWeight(particles_ptr_);
    return true;
}

bool ParticleFilter::resample(std::shared_ptr<Particles> new_particles_ptr, const size_t particle_num,
                              const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen)
{
    if (particles_ptr_->empty() || particle_num == 0)
        return false;

    std::vector<size_t> indices = drawSystematicIndices(particle_num);
    // Systematic sampling returns sorted indices. Shuffle them so that the particles replaced by
    // random particles are not biased towards the tail of the particle vector.
    std::shuffle(indices.begin(), indices.end(), rand_);

    const double ratio = std::min(std::max(random_particle_ratio, 0.0), 1.0);
    const size_t resample_particle_num = (size_t)((double)particle_num * (1.0 - ratio) + 0.5);
    new_particles_ptr->clear();
    new_particles_ptr->reserve(particle_num);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const State &source = particles_ptr_->at(indices.at(i));
        if (i < resample_particle_num)
        {
            State new_state = source;
            new_state.weight = 1.0;
            new_particles_ptr->push_back(new_state);
        }
        else
        {
            new_particles_ptr->push_back(addNoise(source, random_particle_noise_gen));
        }
    }
    return true;
}

State ParticleFilter::getMMSE()
{
    State mmse_state;
    mmse_state.position = Position::Zero();
    mmse_state.quat = Quat::Identity();
    mmse_state.weight = 0.0;
    if (particles_ptr_->empty())
        return mmse_state;

    double weight_sum = 0.0;
    for (const auto &particle : *particles_ptr_)
        weight_sum += particle.weight;
    const bool uniform = !(weight_sum > 0.0) || !std::isfinite(weight_sum);
    mmse_state.weight = weight_sum;

    // position: weighted mean
    // orientation: weighted quaternion average (Markley et al., 2007), i.e. the eigenvector of
    // M = sum(w * q * q^T) with the largest eigenvalue. It is free from Euler angle ambiguity/wrapping.
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    for (const auto &particle : *particles_ptr_)
    {
        const double w = uniform ? 1.0 / (double)particles_ptr_->size() : particle.weight / weight_sum;
        mmse_state.position += w * particle.position;
        const Eigen::Vector4d q = particle.quat.normalized().coeffs(); // x,y,z,w
        m += w * q * q.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(m);
    const Eigen::Vector4d q_avg = solver.eigenvectors().col(3); // eigenvalues are sorted in increasing order
    mmse_state.quat = Quat(q_avg(3), q_avg(0), q_avg(1), q_avg(2)).normalized();
    if (mmse_state.quat.w() < 0.0)
        mmse_state.quat.coeffs() *= -1.0;

    return mmse_state;
}

State ParticleFilter::getMAP()
{
    if (particles_ptr_->empty())
    {
        State state;
        state.position = Position::Zero();
        state.quat = Quat::Identity();
        state.weight = 0.0;
        return state;
    }
    return *std::max_element(particles_ptr_->begin(), particles_ptr_->end(),
                             [](const State &a, const State &b) { return a.weight < b.weight; });
}

bool ParticleFilter::predict(std::shared_ptr<PredictionModelInterface> model, const Time &time)
{
    return predict(particles_ptr_, model, time);
}

bool ParticleFilter::predict(std::shared_ptr<Particles> particles_ptr, std::shared_ptr<PredictionModelInterface> model,
                             const Time &time, bool update_time)
{
    const double dt_sec = Time::getDiff(last_prediction_time_, time);
    if (update_time && dt_sec < 0.0)
        return false; // stale time stamp, the internal clock must not go backwards
    if (dt_sec != 0.0)
    {
        for (auto &particle : *particles_ptr)
        {
            model->predict(particle, dt_sec);
        }
    }
    if (update_time)
        last_prediction_time_ = time;
    return true;
}

bool ParticleFilter::measure(std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state)
{
    return measure(particles_ptr_, model, measuremnt_state);
}

bool ParticleFilter::measure(std::shared_ptr<const Particles> measurement_point_particles_ptr,
                             std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state)
{
    return model->measure(measurement_point_particles_ptr, particles_ptr_, measuremnt_state);
}

bool ParticleFilter::getParticles(std::shared_ptr<const Particles> &particles_ptr)
{
    particles_ptr = particles_ptr_;
    return true;
}

void ParticleFilter::normalizeWeight()
{
    normalizeWeight(particles_ptr_);
}

void ParticleFilter::normalizeWeight(std::shared_ptr<Particles> particles_ptr)
{
    if (particles_ptr->empty())
        return;
    double weight_sum = 0.0;
    for (const auto &particle : *particles_ptr)
    {
        weight_sum += particle.weight;
    }
    if (!(weight_sum > 0.0) || !std::isfinite(weight_sum))
    {
        // all weights underflowed (or are invalid): fall back to uniform weights
        const double uniform_weight = 1.0 / (double)particles_ptr->size();
        for (auto &particle : *particles_ptr)
            particle.weight = uniform_weight;
        return;
    }
    for (auto &particle : *particles_ptr)
    {
        particle.weight /= weight_sum;
    }
}

size_t ParticleFilter::getParticleNum()
{
    return particles_ptr_->size();
}

double ParticleFilter::getESS(const bool normalized) // effective sample size
{
    // in order to calc effective sample size, particles must be normalized weight
    double weight_square_sum = 0.0;
    if (!normalized)
    {
        double weight_sum = 0.0;
        for (const auto &particle : *particles_ptr_)
        {
            weight_sum += particle.weight;
        }
        const double denominator = weight_sum * weight_sum;
        for (const auto &particle : *particles_ptr_)
        {
            weight_square_sum += particle.weight * particle.weight / denominator;
        }
    }
    else
    {
        for (const auto &particle : *particles_ptr_)
        {
            weight_square_sum += particle.weight * particle.weight;
        }
    }
    if (weight_square_sum <= 0.0)
        return std::numeric_limits<double>::max();
    return 1.0 / weight_square_sum;
}

double ParticleFilter::getNormalQuantile(const double p)
{
    // Peter J. Acklam's rational approximation (relative error < 1.15e-9)
    constexpr double a[6] = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
                             1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
    constexpr double b[5] = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
                             6.680131188771972e+01, -1.328068155288572e+01};
    constexpr double c[6] = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
                             -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
    constexpr double d[4] = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
                             3.754408661907416e+00};
    constexpr double p_low = 0.02425;
    constexpr double p_high = 1.0 - p_low;

    if (p <= 0.0)
        return -std::numeric_limits<double>::infinity();
    if (p >= 1.0)
        return std::numeric_limits<double>::infinity();
    if (p < p_low)
    {
        const double q = std::sqrt(-2.0 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    if (p <= p_high)
    {
        const double q = p - 0.5;
        const double r = q * q;
        return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
               (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
    }
    const double q = std::sqrt(-2.0 * std::log(1.0 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
           ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
}

} // namespace amcl_3d
