// Asynchronous and delayed inputs: the localizer must give the same result as if every input had
// arrived in time order, as long as the delay is within max_measurement_delay.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
Time seconds(double s) { return Time::fromSeconds(s); }

/// One input: applied to a localizer at `stamp`, delivered at `arrival`.
struct Event
{
  double stamp;
  double arrival;
  std::function<void(Localizer &, const Time &)> apply;
};

LocalizerParams noisyParams(uint64_t seed)
{
  LocalizerParams params;
  params.random_seed = seed;
  params.initial_particle_num = 300;
  params.max_measurement_delay = 1.0;
  return params;  // default motion noise and resampling: every step draws random numbers
}

/// Feeds the events ordered by `key` into a fresh localizer.
Localizer run(
  const LocalizerParams & params, std::vector<Event> events,
  const std::function<double(const Event &)> & key,
  const std::function<void(Localizer &)> & setup = {}, const Pose & initial_pose = Pose{})
{
  std::stable_sort(events.begin(), events.end(), [&](const Event & a, const Event & b) {
    return key(a) < key(b);
  });
  Localizer localizer(params);
  if (setup) {
    setup(localizer);
  }
  PoseWithCovariance initial;
  initial.pose = initial_pose;
  initial.covariance.diagonal() << 0.5, 0.5, 0.01, 0.001, 0.001, 0.05;
  localizer.initialize(initial, seconds(0.0));
  for (const auto & event : events) {
    event.apply(localizer, seconds(event.stamp));
  }
  return localizer;
}

void expectIdentical(const Localizer & actual, const Localizer & expected)
{
  EXPECT_EQ(actual.stamp(), expected.stamp());
  ASSERT_EQ(actual.particles().size(), expected.particles().size());
  ASSERT_GT(expected.particles().size(), 0u);  // not trivially equal
  EXPECT_EQ(actual.particles().positions(), expected.particles().positions());
  EXPECT_EQ(actual.particles().orientations(), expected.particles().orientations());
  EXPECT_EQ(actual.particles().logWeights(), expected.particles().logWeights());
}

Twist twistAt(double t)
{
  Twist twist;
  twist.linear.x() = 2.0 + std::sin(t);
  twist.angular.z() = 0.2 * std::cos(2.0 * t);
  return twist;
}

std::shared_ptr<const MeasurementModel> beaconsAt(double t)
{
  const Eigen::Vector3d position(2.0 * t, 0.3 * t, 0.0);  // roughly the driven path
  std::vector<Eigen::Vector3d> beacons{{20.0, 0.0, 0.0}, {0.0, 20.0, 0.0}, {-15.0, -15.0, 0.0}};
  std::vector<double> ranges;
  for (const auto & beacon : beacons) {
    ranges.push_back((position - beacon).norm());
  }
  return std::make_shared<test::BeaconRangeModel>(beacons, ranges, 0.3);
}

/// Odometry at 20 Hz (a few twists 0.3 s late), beacons at 10 Hz (20 ms late) and an absolute
/// pose sensor at 3 Hz delivered 0.6 s late, like a slow GNSS or a heavy detector.
std::vector<Event> makeEvents(SensorId beacon_sensor)
{
  std::vector<Event> events;
  for (int k = 0; k < 60; ++k) {
    const double t = 0.05 * k;
    const double delay = k % 7 == 3 ? 0.3 : 0.005;
    events.push_back(
      {t, t + delay, [t](Localizer & l, const Time & s) { l.addTwist(twistAt(t), s); }});
  }
  for (int k = 1; k < 30; ++k) {
    const double t = 0.1 * k + 0.01;
    events.push_back({t, t + 0.02, [beacon_sensor, t](Localizer & l, const Time & s) {
                        ASSERT_EQ(l.update(beacon_sensor, beaconsAt(t), s), LocalizerStatus::kOk);
                      }});
  }
  for (int k = 1; k < 9; ++k) {
    const double t = 0.33 * k;
    events.push_back({t, t + 0.6, [t](Localizer & l, const Time & s) {
                        PoseWithCovariance observation;
                        observation.pose.position = Eigen::Vector3d(2.0 * t, 0.3 * t, 0.0);
                        observation.covariance.diagonal() << 0.25, 0.25, 0.25, 0.1, 0.1, 0.1;
                        ASSERT_EQ(l.updateWithPose(observation, s), LocalizerStatus::kOk);
                      }});
  }
  return events;
}
/// log p = -0.5 (coordinate / sigma)^2 of one coordinate of the particle positions.
class AxisModel : public MeasurementModel
{
public:
  AxisModel(int axis, double sigma) : axis_(axis), sigma_(sigma) {}
  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
  {
    LogLikelihood result;
    result.values = -0.5 * (particles.positions().col(axis_).array() / sigma_).square().matrix();
    return result;
  }

private:
  int axis_;
  double sigma_;
};

/// Rules out the particles with x outside (min, max).
class XIntervalModel : public MeasurementModel
{
public:
  XIntervalModel(double min, double max) : min_(min), max_(max) {}
  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
  {
    const Eigen::ArrayXd x = particles.positions().col(0).array();
    LogLikelihood result;
    result.values = (x > min_ && x < max_).select(0.0, Eigen::ArrayXd::Constant(x.size(), -1000.0));
    return result;
  }

private:
  double min_;
  double max_;
};

LocalizerParams staticParams(uint64_t seed)
{
  LocalizerParams params = noisyParams(seed);
  params.motion_model.twist = {0.0, 0.0, 0.0, 0.0, 1};  // particles keep their poses
  params.resampling.ess_ratio_threshold = 1.0;          // resample at every update
  params.max_measurement_delay = 2.0;
  params.pose_sensor.delayed_fusion = DelayedFusion::kReweight;
  return params;
}
}  // namespace

TEST(AsynchronousLocalizer, ReweightingFollowsTheGenealogy)
{
  // Updates on y at 10 Hz resample (copy) the particles, which move without noise. Measurements
  // on x arrive after all of them and are reweighted onto the descendants of the particles at
  // their time: the weight must end up on particles that were inside every x interval only.
  struct Scenario
  {
    const char * name;
    std::vector<Event> late;
    double min;
    double max;
    double speed = 0.0;        // forward [m/s]
    double gate_stamp = 0.35;  // of the interval measurements, when moving
  };
  SensorId lateral = 0;
  SensorId gate = 0;
  const auto lateral_update = [&lateral](Localizer & l, const Time & s) {
    ASSERT_EQ(l.update(lateral, std::make_shared<AxisModel>(1, 0.5), s), LocalizerStatus::kOk);
  };
  const auto gate_update = [&gate](double min, double max) {
    return [&gate, min, max](Localizer & l, const Time & s) {
      ASSERT_EQ(
        l.update(gate, std::make_shared<XIntervalModel>(min, max), s), LocalizerStatus::kOk);
    };
  };
  const std::vector<Scenario> scenarios{
    {"one", {{0.35, 2.0, gate_update(0.0, 10.0)}}, 0.0, 10.0},
    // the second goes through the genealogy of the newest state resampled by the first
    {"two", {{0.35, 2.0, gate_update(0.0, 10.0)}, {0.75, 2.1, gate_update(-10.0, 0.8)}}, 0.0, 0.8},
    // a replay from a state between the measurement and the newest one keeps its weight
    {"replayed", {{0.35, 2.0, gate_update(0.0, 10.0)}, {1.45, 2.1, lateral_update}}, 0.0, 10.0},
    // scored at 0.35 s on the path between the states at 0.3 s and 0.4 s
    {"moving", {{0.35, 2.0, gate_update(0.0, 0.5)}}, 0.0, 0.5, 1.0},
  };
  for (const uint64_t seed : {1, 2, 3}) {
    for (const auto & scenario : scenarios) {
      const LocalizerParams params = staticParams(seed);
      const auto setup = [&](Localizer & l) {
        lateral = l.addSensor({"lateral"});
        gate = l.addSensor({"gate", 0, false, 0.0, DelayedFusion::kReweight});
      };
      std::vector<Event> events = scenario.late;
      if (scenario.speed > 0.0) {
        Twist twist;
        twist.linear.x() = scenario.speed;
        events.push_back(
          {0.0, 0.0, [twist](Localizer & l, const Time & s) { l.addTwist(twist, s); }});
      }
      for (int k = 1; k <= 15; ++k) {
        events.push_back({0.1 * k, 0.1 * k, lateral_update});
      }
      const Localizer in_order = run(
        params, events, [](const Event & e) { return e.stamp; }, setup);
      const Localizer late = run(
        params, events, [](const Event & e) { return e.arrival; }, setup);
      EXPECT_GE(late.reweightedCount(), 1u);

      const ParticleSet & particles = late.particles();
      const Eigen::VectorXd weights = particles.weights();
      size_t weighted = 0;
      for (size_t i = 0; i < particles.size(); ++i) {
        if (weights(static_cast<Eigen::Index>(i)) > 1e-6) {
          // straight motion without noise: back to the measurement time
          const Pose pose = particles.pose(i);
          const double x = pose.position.x() - (pose.orientation * Eigen::Vector3d::UnitX()).x() *
                                                 scenario.speed *
                                                 (late.stamp().seconds() - scenario.gate_stamp);
          EXPECT_TRUE(x > scenario.min && x < scenario.max)
            << scenario.name << ", seed " << seed << ", particle " << i << ": x " << x;
          ++weighted;
        }
      }
      EXPECT_GT(weighted, particles.size() / 10) << scenario.name << ", seed " << seed;
      // the same distribution as in time order, up to the sampling noise of the resampling
      EXPECT_NEAR(late.estimate().pose.position.x(), in_order.estimate().pose.position.x(), 0.15)
        << scenario.name << ", seed " << seed;
    }
  }
}

TEST(AsynchronousLocalizer, ReweightingFallsBackToReplayAcrossItsOwnBranch)
{
  const LocalizerParams params = staticParams(17);
  Localizer localizer(params);
  const SensorId gate = localizer.addSensor({"gate", 0, false, 0.0, DelayedFusion::kReweight});
  localizer.initialize(PoseWithCovariance{}, seconds(0.0));
  for (int k = 1; k <= 10; ++k) {
    ASSERT_EQ(
      localizer.update(
        Localizer::kPointCloudSensor, std::make_shared<AxisModel>(1, 0.5), seconds(0.1 * k)),
      LocalizerStatus::kOk);
  }
  const auto model = std::make_shared<AxisModel>(0, 1.0);
  ASSERT_EQ(localizer.update(gate, model, seconds(0.55)), LocalizerStatus::kOk);
  EXPECT_EQ(localizer.reweightedCount(), 1u);
  EXPECT_EQ(localizer.replayedCount(), 0u);
  // The state at 0.6 s was computed from the one at 0.5 s, not from the measurement at 0.55 s
  // now before it: its genealogy does not lead back to the state before 0.57 s. Replay 0.6 .. 1.0.
  ASSERT_EQ(localizer.update(gate, model, seconds(0.57)), LocalizerStatus::kOk);
  EXPECT_EQ(localizer.reweightedCount(), 1u);
  EXPECT_EQ(localizer.replayedCount(), 5u);
  // which made the history consistent again
  ASSERT_EQ(localizer.update(gate, model, seconds(0.52)), LocalizerStatus::kOk);
  ASSERT_EQ(localizer.update(gate, model, seconds(0.58)), LocalizerStatus::kOk);
  EXPECT_EQ(localizer.reweightedCount(), 3u);
  EXPECT_EQ(localizer.replayedCount(), 5u);
}

TEST(AsynchronousLocalizer, ReweightingFusesLongDelaysWithoutReplaying)
{
  double error_sum = 0.0;
  double in_order_error_sum = 0.0;
  for (const uint64_t seed : {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}) {
    LocalizerParams params = noisyParams(seed);
    params.max_measurement_delay = 2.0;
    params.pose_sensor.delayed_fusion = DelayedFusion::kReweight;
    SensorId beacon_sensor = 0;
    const auto setup = [&](Localizer & l) { beacon_sensor = l.addSensor({"beacons"}); };
    Localizer probe(params);
    setup(probe);
    std::vector<Event> events = makeEvents(beacon_sensor);
    for (auto & event : events) {
      if (event.arrival - event.stamp > 0.5) {
        event.arrival = event.stamp + 1.5;  // the pose sensor: 15 beacon updates late
      } else {
        event.arrival = event.stamp;
      }
    }
    const Localizer in_order = run(
      params, events, [](const Event & e) { return e.stamp; }, setup);
    const Localizer late = run(
      params, events, [](const Event & e) { return e.arrival; }, setup);
    EXPECT_EQ(late.reweightedCount(), 8u);
    EXPECT_EQ(late.replayedCount(), 0u);
    EXPECT_EQ(late.stamp(), in_order.stamp());
    // as accurate as time order: on the path at the last beacon (x = 2 t, y = 0.3 t)
    const Eigen::Vector3d truth(5.82, 0.873, 0.0);
    const double error = (late.estimate().pose.position - truth).norm();
    EXPECT_LT(error, 0.5) << "seed " << seed;
    error_sum += error;
    in_order_error_sum += (in_order.estimate().pose.position - truth).norm();
  }
  // measured 0.151 m and 0.140 m (0.217 m without the pose sensor); the filter's own standard
  // deviation is 0.14 m
  EXPECT_LT(error_sum, 1.5 * in_order_error_sum);
}

TEST(AsynchronousLocalizer, DelayedInputsGiveTheResultOfTimeOrder)
{
  for (const uint64_t seed : {1, 2, 3}) {
    const LocalizerParams params = noisyParams(seed);
    SensorId beacon_sensor = 0;
    const auto setup = [&](Localizer & l) { beacon_sensor = l.addSensor({"beacons"}); };
    Localizer probe(params);
    setup(probe);  // ids are assigned in order, so every run gets the same one
    const std::vector<Event> events = makeEvents(beacon_sensor);

    const Localizer in_order = run(
      params, events, [](const Event & e) { return e.stamp; }, setup);
    const Localizer delayed = run(
      params, events, [](const Event & e) { return e.arrival; }, setup);
    EXPECT_EQ(in_order.replayedCount(), 0u);
    EXPECT_GT(delayed.replayedCount(), 0u);
    expectIdentical(delayed, in_order);
    // and the result is a sensible one: on the path (x = 2 t, y = 0.3 t) at the last beacon
    EXPECT_EQ(in_order.stamp(), seconds(2.91));
    EXPECT_LT((in_order.estimate().pose.position - Eigen::Vector3d(5.82, 0.873, 0.0)).norm(), 0.5)
      << "seed " << seed;
  }
}

TEST(AsynchronousLocalizer, LateTwistReplaysTheIntervalItCovers)
{
  const LocalizerParams params = noisyParams(5);
  std::vector<Event> events;
  for (int k = 1; k <= 10; ++k) {
    const double t = 0.1 * k;
    events.push_back({t, t, [](Localizer & l, const Time & s) { l.predict(s); }});
  }
  events.push_back({0.25, 0.95, [](Localizer & l, const Time & s) {
                      EXPECT_EQ(l.addTwist(twistAt(0.0), s), LocalizerStatus::kOk);
                    }});
  const Localizer in_order = run(params, events, [](const Event & e) { return e.stamp; });
  const Localizer late = run(params, events, [](const Event & e) { return e.arrival; });
  // the twist (2 m/s from 0.25 s) moved the vehicle about 1.5 m
  EXPECT_NEAR(in_order.estimate().pose.position.x(), 1.5, 0.3);
  EXPECT_EQ(late.replayedCount(), 7u);  // the predictions at 0.3 .. 0.9 (1.0 comes after it)
  expectIdentical(late, in_order);
}

TEST(AsynchronousLocalizer, BoundsTheHistoryAndRejectsOlderInputs)
{
  LocalizerParams params = noisyParams(7);
  params.max_measurement_delay = 0.5;
  Localizer localizer(params);
  localizer.initialize(PoseWithCovariance{}, seconds(0.0));
  for (int k = 1; k <= 30; ++k) {
    ASSERT_EQ(localizer.predict(seconds(0.1 * k)), LocalizerStatus::kOk);
    EXPECT_LE(localizer.historySize(), 6u);  // entries within 0.5 s of the newest
  }
  const ParticleSet before = localizer.particles();
  EXPECT_EQ(localizer.predict(seconds(2.45)), LocalizerStatus::kStaleStamp);
  EXPECT_EQ(localizer.particles().positions(), before.positions());
  EXPECT_EQ(localizer.predict(seconds(2.65)), LocalizerStatus::kOk);  // delayed, within 0.5 s
  EXPECT_EQ(localizer.stamp(), seconds(3.0));
  // before the history, but the twist at its start: applies from there on
  EXPECT_EQ(localizer.addTwist(twistAt(0.0), seconds(2.0)), LocalizerStatus::kOk);
  // superseded by the twist at 2 s before the history starts: never used
  EXPECT_EQ(localizer.addTwist(twistAt(0.0), seconds(1.0)), LocalizerStatus::kStaleStamp);

  params.max_measurement_delay = 0.0;  // no history: only time order
  Localizer strict(params);
  strict.initialize(PoseWithCovariance{}, seconds(0.0));
  ASSERT_EQ(strict.predict(seconds(1.0)), LocalizerStatus::kOk);
  EXPECT_EQ(strict.historySize(), 0u);
  EXPECT_EQ(strict.predict(seconds(0.9)), LocalizerStatus::kStaleStamp);
  EXPECT_EQ(strict.predict(seconds(1.0)), LocalizerStatus::kOk);  // same stamp is in order
}

TEST(AsynchronousLocalizer, DelayedScansGiveTheResultOfTimeOrder)
{
  const PointCloud::Ptr map = test::makeRoomMap();
  Pose true_pose;
  true_pose.position = Eigen::Vector3d(3.0, 2.0, 0.0);
  true_pose.orientation = test::yawQuaternion(0.3);
  const PointCloud::Ptr scan = test::makeScan(*map, true_pose);
  const auto likelihood = std::make_shared<LikelihoodField>(
    std::make_shared<DistanceFieldMap>(*map, DistanceFieldMap::Params{0.1, 1.0}),
    LikelihoodField::Params{});

  LocalizerParams params = noisyParams(11);
  params.scan_filter.voxel_size = 0.3;
  params.scan_sample_num = 100;
  std::vector<Event> events;
  for (int k = 1; k <= 12; ++k) {
    const double t = 0.1 * k;
    // every third scan arrives 0.25 s late, behind two newer ones
    events.push_back({t, k % 3 == 0 ? t + 0.25 : t, [&scan](Localizer & l, const Time & s) {
                        ASSERT_EQ(l.updateWithPointCloud(*scan, s), LocalizerStatus::kOk);
                      }});
  }
  const auto setup = [&](Localizer & l) { l.setPointCloudLikelihood(likelihood); };
  for (const size_t annealing_layer_num : {0, 2}) {  // annealing resamples and moves in between
    params.point_cloud_sensor.annealing_layer_num = annealing_layer_num;
    const Localizer in_order = run(
      params, events, [](const Event & e) { return e.stamp; }, setup, true_pose);
    const Localizer delayed = run(
      params, events, [](const Event & e) { return e.arrival; }, setup, true_pose);
    EXPECT_GT(delayed.replayedCount(), 0u);
    expectIdentical(delayed, in_order);
    EXPECT_LT((in_order.estimate().pose.position - true_pose.position).norm(), 0.5);
  }
}

TEST(AsynchronousLocalizer, OnlySensorsThatDriveRecoveryTriggerResets)
{
  // an observation that no particle explains (per-observation likelihood e^-50)
  class Unexplained : public MeasurementModel
  {
  public:
    LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
    {
      return LogLikelihood{
        Eigen::VectorXd::Constant(static_cast<Eigen::Index>(particles.size()), -50.0), 1};
    }
  };
  LocalizerParams params = noisyParams(13);
  params.expansion_reset.enabled = true;
  Localizer localizer(params);
  const SensorId passive = localizer.addSensor({"passive", 0, false, 0.05});
  const SensorId watched = localizer.addSensor({"watched", 0, true, 0.05});
  localizer.initialize(PoseWithCovariance{}, seconds(0.0));
  const auto model = std::make_shared<Unexplained>();

  ASSERT_EQ(localizer.update(passive, model, seconds(0.1)), LocalizerStatus::kOk);
  EXPECT_EQ(localizer.expansionResetCount(), 0u);
  ASSERT_EQ(localizer.update(watched, model, seconds(0.2)), LocalizerStatus::kOk);
  EXPECT_EQ(localizer.expansionResetCount(), 1u);
  // pose observations (e.g. GNSS) do not drive recovery by default
  EXPECT_FALSE(localizer.sensor(Localizer::kPoseSensor).drives_recovery);
  EXPECT_FALSE(localizer.sensor(Localizer::kScanMatchingSensor).drives_recovery);
  EXPECT_TRUE(localizer.sensor(Localizer::kPointCloudSensor).drives_recovery);
}

}  // namespace amcl_3d
