#include "amcl_3d/resample_noise/normal_distribution.hpp"
#include <cmath>

namespace amcl_3d
{
// NOTE: `var` is a variance. std::normal_distribution takes a standard deviation, so sqrt is applied.
// A non-positive variance yields the average without noise (std::normal_distribution requires stddev > 0).
NormalDistribution::NormalDistribution(const double avg, const double var)
    : avg_(avg), var_(var), noise_(avg_, var_ > 0.0 ? std::sqrt(var_) : 1.0) {}
double NormalDistribution::getNoise(XorShift128 &rand)
{
    if (var_ <= 0.0)
        return avg_;
    return noise_(rand);
}
void NormalDistribution::setVar(const double var)
{
    var_ = var;
    noise_ = std::normal_distribution<double>(avg_, var_ > 0.0 ? std::sqrt(var_) : 1.0);
}
void NormalDistribution::setAvg(const double avg)
{
    avg_ = avg;
    noise_ = std::normal_distribution<double>(avg_, var_ > 0.0 ? std::sqrt(var_) : 1.0);
}
double NormalDistribution::getVar() { return var_; }
double NormalDistribution::getAvg() { return avg_; }
} // namespace amcl_3d
