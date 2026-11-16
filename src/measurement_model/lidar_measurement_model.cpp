#include "amcl_3d/measurement_model/lidar_measurement_model.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace amcl_3d
{
XorShift128 LidarMeasurementModel::rand_;

LidarMeasurementModel::LidarMeasurementModel(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr measurement_ptr,
                                             const size_t random_sample_num,
                                             const double max_dist,
                                             const double sigma)
    : kd_map_ptr_(kd_map_ptr),
      measurement_ptr_(measurement_ptr),
      max_dist_(max_dist),
      sigma_(sigma)
{
    updateRandomSampleIndexVec(random_sample_num);
}

bool LidarMeasurementModel::measure(std::shared_ptr<const Particles> mesurement_point_particles_ptr,
                                    std::shared_ptr<Particles> particles_ptr,
                                    MeasurementState &measurement_state)
{
    if (!isKdMap() || measurement_ptr_ == nullptr || v_random_sample_index_.empty() ||
        mesurement_point_particles_ptr->empty() || particles_ptr->empty())
        return false;
    if (particles_ptr->size() != mesurement_point_particles_ptr->size())
    {
        std::cerr << "Error:" << __FILE__ << ", line:" << __LINE__
                  << " measurement particles and particles have different size" << std::endl;
        return false;
    }

    // calc log likelihood each particle
    std::vector<double> v_log_likelihood;
    v_log_likelihood.reserve(particles_ptr->size());
    const double max_squared_dist = max_dist_ * max_dist_; // [m^2]
    const double denominator = 2 * sigma_ * sigma_;
    constexpr int k = 1;
    std::vector<int> v_point_idx(k);
    std::vector<float> v_squared_dist(k);
    for (const auto &state : *mesurement_point_particles_ptr)
    {
        /*
         * likelihood = exp(-(x-mu)^2/(2*sigma^2)), x is variable, mu is avg
         * log likelihood = -(x-mu)^2/(2*sigma^2)
         */
        double log_likelihood = 0.0;
        for (const auto &i : v_random_sample_index_)
        {
            // transform measurement point (base_link frame) into map frame by particle pose
            const Position local_point(measurement_ptr_->at(i).x, measurement_ptr_->at(i).y, measurement_ptr_->at(i).z);
            const Position point = state.quat * local_point + state.position;
            const pcl::PointXYZ sample_point(point.x(), point.y(), point.z());
            // nearest neighbor search
            double squared_dist;
            if (kd_map_ptr_->nearestKSearch(sample_point, k, v_point_idx, v_squared_dist) > 0)
                squared_dist = std::min((double)v_squared_dist.at(0), max_squared_dist);
            else
                squared_dist = max_squared_dist;
            log_likelihood += -1.0 * squared_dist;
        }
        log_likelihood /= denominator; // denominator = (2*sigma^2)
        v_log_likelihood.push_back(log_likelihood);
    }

    // calc weight and avoid numerical underflow
    // http://www.maths.lu.se/fileadmin/maths/forskning_research/InferPartObsProcess/particlemethods.pdf
    const double max_log_likelihood = *std::max_element(v_log_likelihood.begin(), v_log_likelihood.end());
    for (size_t i = 0; i < particles_ptr->size(); ++i)
    {
        particles_ptr->at(i).weight *= std::exp(v_log_likelihood.at(i) - max_log_likelihood);
    }
    // calc raw weight average for augmented mcl.
    // The joint likelihood of all points underflows easily, so the per-point (geometric mean) likelihood is used.
    const double sample_num = (double)v_random_sample_index_.size();
    double weight_sum = 0.0;
    for (size_t i = 0; i < particles_ptr->size(); ++i)
    {
        weight_sum += std::exp(v_log_likelihood.at(i) / sample_num);
    }
    measurement_state.raw_weight_avg = weight_sum / (double)particles_ptr->size();
    return true;
}

void LidarMeasurementModel::updateRandomSampleIndexVec(const size_t random_sample_num)
{
    v_random_sample_index_.clear();
    if (measurement_ptr_ == nullptr || measurement_ptr_->empty())
        return;
    const size_t limited_random_sample_num = std::min(random_sample_num, measurement_ptr_->size());
    v_random_sample_index_.reserve(limited_random_sample_num);
    // draw indices from the whole measurement cloud
    std::uniform_int_distribution<size_t> engine(0, measurement_ptr_->size() - 1);
    for (size_t i = 0; i < limited_random_sample_num; ++i)
    {
        v_random_sample_index_.push_back(engine(rand_));
    }
}

void LidarMeasurementModel::setKdMap(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr)
{
    kd_map_ptr_ = kd_map_ptr;
}

void LidarMeasurementModel::setMeasuement(pcl::PointCloud<pcl::PointXYZ>::Ptr measurement_ptr)
{
    const size_t random_sample_num = v_random_sample_index_.size();
    measurement_ptr_ = measurement_ptr;
    updateRandomSampleIndexVec(random_sample_num);
}

bool LidarMeasurementModel::isKdMap()
{
    if (kd_map_ptr_ == nullptr)
        return false;
    return true;
}

void LidarMeasurementModel::setRandomSampleNum(const size_t random_sample_num)
{
    updateRandomSampleIndexVec(random_sample_num);
}

size_t LidarMeasurementModel::getRandomSampleNum()
{
    return v_random_sample_index_.size();
}
} // namespace amcl_3d
