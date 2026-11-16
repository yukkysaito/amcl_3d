#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include <cmath>

namespace amcl_3d
{
XorShift128 FooPredictionModel::rand_;
FooPredictionModel::FooPredictionModel() : vel_(Eigen::Vector3d::Zero()), omega_(Eigen::Vector3d::Zero()) {}
FooPredictionModel::FooPredictionModel(const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &omega)
    : vel_(vel), omega_(omega)
{
}
FooPredictionModel::FooPredictionModel(const Eigen::Vector3d &vel, const Eigen::Vector3d &omega,
                                       const NoiseParam &noise_param)
    : vel_(vel), omega_(omega), noise_param_(noise_param)
{
}

bool FooPredictionModel::predict(State &state, const double dt_sec)
{
    if (dt_sec == 0.0)
        return true;

    double vel = vel_.x();
    double omega = omega_.z();
    // Add noise only when moving forward in time. Backward prediction (dt < 0) is deterministic.
    if (0.0 < dt_sec)
    {
        std::normal_distribution<double> standard_normal(0.0, 1.0);
        const double inv_sqrt_dt = 1.0 / std::sqrt(dt_sec);
        vel += (noise_param_.vel_scale * std::abs(vel) + noise_param_.vel_bias) * inv_sqrt_dt * standard_normal(rand_);
        omega += (noise_param_.omega_scale * std::abs(omega) + noise_param_.omega_bias) * inv_sqrt_dt * standard_normal(rand_);
    }

    const double dyaw = omega * dt_sec;
    Position local_position;
    if (std::abs(omega) < 1e-6)
    {
        // straight motion (avoid r = vel / omega -> inf)
        local_position << vel * dt_sec, 0.0, 0.0;
    }
    else
    {
        const double r = vel / omega;
        local_position << r * std::sin(dyaw), r * (1.0 - std::cos(dyaw)), 0.0;
    }
    const Quat local_quat(Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()));

    const Quat quat = state.quat.normalized();
    state.position = state.position + quat * local_position;
    state.quat = (quat * local_quat).normalized();
    return true;
}

bool FooPredictionModel::measumentLinearVelocity(const Eigen::Vector3d &vel)
{
    vel_ = vel;
    return true;
}
bool FooPredictionModel::measumentAngularVelocity(const Eigen::Vector3d &omega)
{
    omega_ = omega;
    return true;
}

Eigen::Vector3d FooPredictionModel::getLinearVelocity() { return vel_; }
Eigen::Vector3d FooPredictionModel::getAngularVelocity() { return omega_; }

} // namespace amcl_3d
