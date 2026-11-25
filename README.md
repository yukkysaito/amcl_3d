# amcl_3d

3D Monte Carlo localization (particle filter) of a vehicle in a point cloud map with a LiDAR, for ROS 2.

## Packages

| package | description |
|---|---|
| [`amcl_3d_core`](amcl_3d_core) | ROS independent C++17 library (Eigen, PCL). Particle filter, motion / measurement models, resamplers. |
| [`amcl_3d`](amcl_3d) | ROS 2 node (`rclcpp_components`) wrapping the library. |

## Build

Built and tested with ROS 2 Humble and Jazzy (see [CI](.github/workflows/ci.yml)).

```bash
cd <ros2_ws>/src
git clone https://github.com/yukkysaito/amcl_3d.git
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select amcl_3d_core amcl_3d && colcon test-result --verbose
```

The core library can also be built without ROS (only its tests are built then):

```bash
cmake -S amcl_3d_core -B build && cmake --build build && ./build/test_amcl_3d_core
```

### CUDA (optional)

With the CUDA toolkit (tested with 12.8), the likelihood of the distance field can run on the GPU:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMCL3D_USE_CUDA=ON
```

then set `point_cloud_likelihood.likelihood_field.map.device: cuda`. `CMAKE_CUDA_ARCHITECTURES` defaults to the GPU of the build machine
(`native`); set it explicitly (e.g. `"86;89"`) when building elsewhere. Without CUDA, the node
reports the `cuda` device as an error at startup.

## Run

```bash
ros2 launch amcl_3d amcl_3d.launch.py rviz:=true \
  input_map:=/map_points input_points:=/points input_odom:=/odom
```

Then set the initial pose with "2D Pose Estimate" in RViz.

### Topics

| topic | type | |
|---|---|---|
| `map` | `sensor_msgs/PointCloud2` (transient local) | map points (or set `map_pcd_file`) |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | (re)initializes the particles |
| `odom` | `nav_msgs/Odometry` | twist in the base frame, used for prediction |
| `imu` | `sensor_msgs/Imu` | angular velocity, if `use_imu` |
| `points` | `sensor_msgs/PointCloud2` | LiDAR scan, any frame with a TF to the base frame |
| `pose` (out) | `geometry_msgs/PoseWithCovarianceStamped` | estimate in the global frame |
| `particles` (out) | `geometry_msgs/PoseArray` | |
| `tf` (out) | | `map` -> `odom` |

Service `~/global_localization` (`std_srvs/Trigger`) spreads particles over the whole map
(use `resampling.type: kld` with it).

Parameters are documented in [`amcl_3d/config/amcl_3d.param.yaml`](amcl_3d/config/amcl_3d.param.yaml).

## Features

- **Asynchronous, delayed sensors**: measurements and odometry are fused in the order of their
  stamps whatever their arrival order; one that arrives up to `max_measurement_delay` late is
  inserted at its time and the newer measurements are replayed, with the same result as if it had
  arrived in time; or, per sensor, reweighted onto the descendants of the particles at its time
  (retroactive particle filter), at the cost of one update whatever the delay (see Design)
- **Prediction**: odometry twists (optionally with IMU angular velocity) are buffered and integrated
  on SE(3) up to each measurement stamp; white velocity noise independent of the odometry rate,
  with
  - `twist`: generic 6-DoF motion with isotropic noise, or
  - `vehicle`: car-like motion (forward speed and yaw rate, base frame at the rear axle) with
    anisotropic noise growing with the speed; 25 % lower yaw error than `twist` on KITTI with
    wheel-like odometry (see Evaluation)
- **Correction** by a scan likelihood (`point_cloud_likelihood.type`), robust to points not in
  the map by clipping:
  - `likelihood_field` (default): distance from each point to the nearest map point, on
    - `distance_field` (default): precomputed sparse field of nearest map points, ~12x faster
      updates than `kdtree` with the same accuracy on KITTI (see Evaluation), or
    - `kdtree`: exact nearest neighbor search
  - `ndt`: Mahalanobis distance to the normal distributions of the map points in 1 m voxels;
    a 10x smaller map than the distance field, at ~10 % more error on KITTI
  - `gicp`: `ndt` plus the local covariance of the scan around each point (voxelized GICP,
    plane-to-plane); no gain over `ndt` on KITTI, available for other sensors and maps
  - tempered: a scan counts as at most `scan_max_effective_points` (10) independent points,
    since its points are strongly correlated; 25-40 % lower error on KITTI than treating them as
    independent
  - particles are evaluated in parallel on `thread_num` CPU threads (OpenMP), or on a CUDA
    GPU (`device: cuda`, likelihood field), which affords 10-100x more particles
- **Other sensors** plug in as a `MeasurementModel` fused by `Localizer::update()` (see Design)
- **Scan preprocessing**: range / height crop, voxel grid, random subsampling
- **Resampling**: systematic or KLD sampling, triggered by the effective sample size
- **Recovery**: augmented MCL, expansion resetting (statistics per sensor), global localization
- **Tempered SMC** (default): adaptive tempered SMC with MCMC moves (`scan_tempered_smc`;
  random walk, MALA, preconditioned MALA or Gauss-Newton moves), the consistent alternative to
  annealing: the posterior covariance matches the error (NEES ~3), and at the same cost it is
  more accurate (0.081 -> 0.075 m on 6 KITTI sequences)
- **Annealing**: annealed particle filter layers before each scan update
  (`scan_annealing_layers`), which gather the cloud where the scan fits rather than where the
  motion model scattered it
- **Refinement**: optional scan matching of the estimate, GICP (default, cm-level) or NDT (with
  optional feedback)
- **Estimate**: mean and covariance (quaternion averaging) of the most likely cluster of particles
  (`estimate.type`), so that a multimodal cloud, e.g. during global localization, yields a pose on
  its strongest mode rather than between the modes; plus `map` -> `odom`

## Design

```
Localizer                      time handling, resampling policy, recovery, refinement
 ├─ ParticleFilter             initialize / predict / update / resample / estimate
 │   └─ ParticleSet            structure of arrays + log weights (vectorization / GPU friendly)
 ├─ MotionModel                TwistMotionModel, VehicleMotionModel
 ├─ MeasurementModel           one measurement of any sensor, fused by Localizer::update()
 │   ├─ PoseMeasurementModel   absolute pose (external scan matching, GNSS)
 │   └─ PointCloudMeasurementModel
 │       └─ PointCloudLikelihood   sensor model of scans
 │           ├─ LikelihoodField    on PointCloudMap: KdTreeMap, DistanceFieldMap,
 │           │                     CudaDistanceFieldMap (batched over particles x points)
 │           ├─ NdtLikelihood      on GaussianVoxelMap
 │           └─ GicpLikelihood     on GaussianVoxelMap, with scan covariances
 ├─ MeasurementUpdate          per sensor: ImportanceUpdate, AnnealedUpdate, TemperedSmcUpdate
 │   └─ MoveKernel             MCMC moves of tempered SMC: RandomWalkMetropolis, LangevinMove
 │                             (MALA, preconditioned MALA, Gauss-Newton; log-likelihood gradients
 │                             from LikelihoodField on CPU and CUDA, PoseMeasurementModel)
 ├─ Resampler                  SystematicResampler, KldResampler
 ├─ ScanFilter
 └─ ScanMatcher                GicpScanMatcher, NdtScanMatcher
PoseSampler                    GaussianPoseSampler, GroundPoseSampler (global localization)
createMotionModel()            builds the MotionModel selected by MotionModelParams
createPointCloudLikelihood()   builds the likelihood and its map selected by
                               PointCloudLikelihoodParams (type, map type, device)
```

The extension points are interfaces with a factory where a parameter selects the implementation:

- **A new sensor** (camera landmarks, GNSS, UWB, ...) implements `MeasurementModel`: one instance
  holds one measurement and returns the log-likelihood of each particle. `Localizer::update()`
  fuses it with the prediction, resampling and recovery shared by all sensors; the localizer
  needs no change. Long-lived state (a landmark map, camera intrinsics and extrinsics) belongs to a
  sensor model shared by the measurement instances, as `PointCloudLikelihood` is for scans.
- **A new scan likelihood** implements `PointCloudLikelihood`, which owns the map representation
  it needs (distance field, voxel Gaussians, ...), and is added to `createPointCloudLikelihood()`.
  The localizer hands it the whole filtered scan with the sampled points
  (`createMeasurementModel()`), so that per-scan features such as the covariances of GICP are
  computed once per scan.
- **A new map backend** for the likelihood field implements `PointCloudMap` and is added to
  `createPointCloudMap()`; a new motion model implements `MotionModel` and is added to
  `createMotionModel()`.

Per sensor, `SensorParams` sets how many independent observations one update may count as
(tempering) and whether the sensor drives recovery. Augmented MCL and expansion resetting keep
their statistics per recovery sensor: likelihoods of different sensors are not comparable, and
the scan matching feedback (a sensor of its own) reuses a scan and must not count twice.

**Time.** Sensors are asynchronous and some are slow (a GNSS, a heavy detector), so the localizer
keeps the inputs of the last `max_measurement_delay` seconds, each measurement with the filter
state after it (particles, random generator, recovery statistics):

```
base state ─ m1 ─ m2 ─ m3 ─ m4          m2' arrives late (stamp between m2 and m3):
                                         restore the state after m2, insert m2',
base state ─ m1 ─ m2 ─ m2' ─ m3 ─ m4     replay m3 and m4 (prediction and correction)
```

A late twist replays the measurements after its stamp in the same way. Because the random
generator is part of the restored state and scans are sampled from a generator seeded by their
stamp, the result is bit for bit the one of time-ordered arrival (tested on synthetic data and on
the whole KITTI sequence). Inputs older than the history are rejected. The cost is one particle set
of memory per measurement within the delay and, for a late measurement, one update per newer
measurement. A fixed-lag queue was rejected: it would delay every output by the slowest sensor.

Replaying costs delay x update rate updates, e.g. 30 scan updates for a sensor 3 s late. A sensor
can instead use `DelayedFusion::kReweight` (node: `scan_delayed_fusion: reweight`), the
retroactive particle filter of Takeuchi et al. (ROBOMECH 2010): each state also records the parent
of each particle at resampling (its genealogy), so a late measurement is evaluated once on the
particles at its time and its likelihood multiplies the weights of their descendants in every
newer state:

```
state before m2'   p0   p1   p2   p3         m2' scores the particles at its time, on the path from
                    │    ├────┐    │          each parent to its child (interpolated); each newer
state after  m3    p0   p1   p1   p3         particle inherits the score of its ancestor
                    ├────┐    │    │
state after  m4    p0   p0   p1   p3
```

Weights commute, so this is the result of time order as long as the particles are only copied in
between; it is approximate in that the resampling, resets and recovery statistics in between did
not know the measurement. The particles must be scored on their actual paths: scoring fresh
predictions instead (another random path from the same parent) is uncorrelated with the
descendants under a sharp scan likelihood and was worse on KITTI than dropping the scan. When a
genealogy does not lead back to the state before the measurement (a newer state computed from
another branch), the measurement is replayed instead, which also makes the history exact again.

All measurement models follow one convention: the log-likelihood of an observation is <= 0 and 0
when it is perfectly explained, so that `exp(log-likelihood / observation count)` is a likelihood
per observation in (0, 1] comparable across sensors, which augmented MCL and expansion resetting
use.

Source layout of `amcl_3d_core` (headers under `include/amcl_3d_core/`, sources under `src/`):

| directory | contents |
|---|---|
| `.` | `Localizer`, `ParticleFilter`, `ParticleSet`, `ScanFilter`, value types, math, random |
| `map/` | `PointCloudMap` and its implementations, `map_factory`; `src/map/cuda/` holds the CUDA code |
| `measurement_model/`, `point_cloud_likelihood/`, `motion_model/`, `resampler/`, `scan_matcher/`, `pose_sampler/` | one interface and its implementations each |
| `tools/` | KITTI evaluation (`kitti_dataset`, `statistics`) and the likelihood benchmark |

## Evaluation

KITTI odometry sequence 07 (695 m, urban), evaluated with
[`kitti_evaluation`](amcl_3d_core/tools/kitti_evaluation.cpp) (no ROS needed):

- map: even frames registered with the ground truth poses (0.2 m voxels)
- localized: odd frames (5 Hz, 540 frames after a 10 frame warm-up), initial guess off by
  0.5 m / 1.7 deg
- odometry: KITTI has none, so the twist is derived from the ground truth with a 2 % scale error,
  a 0.005 rad/s yaw rate bias and white noise; dead reckoning with it drifts 30-36 m (RMSE)
- defaults otherwise: 500 particles, 200 scan points, `scan_max_effective_points: 10`,
  systematic resampling

| configuration | translation RMSE [m] | yaw RMSE [deg] | max [m] | update mean / p95 [ms] |
|---|---|---|---|---|
| `kdtree` | 0.095-0.107 | 0.12 | 0.46 | 23.1 / 26.4 |
| `distance_field` 0.2 m (default) | 0.092-0.105 | 0.12 | 0.41 | **1.8 / 2.1** |
| `distance_field` 0.1 m | 0.100-0.104 | 0.11-0.13 | 0.46 | 2.4 / 2.9 |
| `distance_field` 0.2 m + NDT | 0.079-0.090 | 0.10 | 0.45 | 36.9 / 63.6 |
| `distance_field` 0.2 m + KLD (~100 particles) | 0.145-0.152 | 0.17 | 0.54 | 1.3 / 1.6 |
| `distance_field` 0.2 m, 10000 particles | 0.068-0.071 | 0.09-0.10 | 0.19 | 15.4 / 17.5 |
| `distance_field` 0.2 m, CUDA | 0.096-0.103 | 0.12 | 0.41 | 1.1 / 1.2 |
| `distance_field` 0.2 m, CUDA, 10000 particles | **0.068-0.070** | **0.09-0.10** | 0.18 | 1.9 / 2.1 |
| `distance_field` 0.2 m, CUDA, 50000 particles | 0.067-0.069 | 0.09 | 0.18 | 5.3 / 5.5 |

Ranges over 3 seeds (over 12 seeds the untempered default spread over 0.153-0.174 m, so
differences below ~0.01 m are noise); timing with `thread_num: 4` on an i9-14900K (the default 500 particles
take 4.1 / 5.0 ms on one thread, 1.5 / 1.8 ms on 8). The results do not depend on the thread
count. No frame was lost (error > 1 m). The distance field takes 4 s to
build and 227 MB (1.2 GB at 0.1 m) for this route.

More particles pay off up to about 10000 (0.07 m, 2 ms per update on the GPU backend, RTX 5080);
50000 no longer improve. With 50000 particles the likelihood takes under 1 ms; prediction runs in
parallel on `thread_num` threads, and the rest is resampling and the estimate on one thread.

Using every hardware thread (`thread_num: 0`, 32 threads) was slower than 4-16 threads
(6.2 ms): idle OpenMP threads spin on the cores the rest of the update runs on. Setting
`OMP_WAIT_POLICY=passive` avoids that.

### Centimeter-level accuracy

The options of [`kitti_accurate.yaml`](amcl_3d_core/tools/config/kitti_accurate.yaml)
(`kitti_evaluation <dataset_dir> 07 config=amcl_3d_core/tools/config/kitti_accurate.yaml`), added
one at a time to the defaults (500 particles, default motion noise), 3 seeds:

| configuration | translation RMSE [m] | mean [m] | p95 [m] | max [m] | frames > 5 cm | update mean / p95 [ms] |
|---|---|---|---|---|---|---|
| defaults | 0.092-0.105 | 0.080-0.089 | 0.16-0.19 | 0.28-0.41 | 73-78 % | 1.9 / 2.7 |
| + 5 annealing layers | 0.061-0.062 | 0.053 | 0.11 | 0.16-0.20 | 45-46 % | 5.6 / 6.3 |
| + `sigma` 0.1 m, 500 scan points | 0.043-0.045 | 0.036-0.038 | 0.08-0.09 | 0.18 | 20-24 % | 11.6 / 13.0 |
| + GICP refinement (`max_correction` 0.1 m) | 0.038-0.040 | 0.030-0.031 | 0.07-0.08 | 0.18-0.22 | 11-13 % | 14.1 / 16.1 |
| the same, `device=cuda` | **0.037-0.039** | **0.029-0.031** | **0.07** | 0.16-0.21 | 11-14 % | **4.3 / 5.8** |
| the same without annealing | 0.129-0.132 | | 0.25 | 0.40-0.52 | | |

- **Annealing** (`SensorParams::annealing_layer_num`, Deutscher et al., 2000): the default motion
  noise spreads the particles by ~0.5 m per 0.2 s at 10 m/s, 8 times the error of the simulated
  odometry, so few particles land where the scan fits. Each layer weights the cloud by the
  likelihood raised to the largest exponent that keeps half of the particles effective, resamples
  and moves the particles by a Gaussian with a quarter of the cloud's covariance. Each part
  matters (5 layers, `sigma` 0.1, 500 points): without the moves 0.15 m, with the exponent fixed to
  1 0.093 m, moves as wide as the cloud 0.20 m. Annealing reaches what matching the motion noise to
  the odometry does by hand (0.042-0.046 m), without knowing the odometry error.
- **Timing**: on the GPU, the 6 likelihood evaluations of an annealed update take 0.03 ms each
  (1.75 ms each on 4 CPU threads); the rest of the 4.3 ms is GICP (2.4 ms), scan filtering (~1 ms)
  and the filter itself (0.6 ms).
- **GICP refinement** (`GicpScanMatcher`): plane-to-plane GICP against the map points with their
  neighborhood covariances, with a Cauchy kernel. Registered from the ground truth offset by
  0.3 m and 1.7 deg, it lands within 2 cm (median); a GICP on 1 m voxel distributions stayed 5-10 cm
  off and PCL's NDT ~0.25 m,
  and NDT refinement instead of GICP gives 0.041-0.042 m. Refinements farther than
  `max_correction` from the particle estimate are rejected: without annealing the estimate is too
  far off for most of them (last row). It registers 1000 points of the scan
  (`GicpScanMatcher::Params::max_point_num`; their covariances still come from the whole scan),
  linearized about the sensor rather than the map origin, in ~8 iterations: 2.4 ms. All ~7500
  points of the filtered scan gave 0.035 m in 14 ms.
- **The reference itself**: registering every localized scan to the map from its ground truth pose
  (PCL GICP) moves it by 0.023 m RMSE (p95 0.039 m), and by 0.1-0.2 m around t = 110 s, where the
  drive closes its loop and the map holds both passes. Of the frames above 5 cm with the full
  configuration, about half lie where the ground truth disagrees with the map by more than 3 cm;
  on the others, 4-6 % of the frames exceed 5 cm.

The node used `scan_annealing_layers: 5` until tempered SMC replaced it (next sections); the
library default is 0.

### Tempered SMC

Benchmark: [`kitti_benchmark.py`](amcl_3d_core/tools/kitti_benchmark.py) with
[`benchmark.yaml`](amcl_3d_core/tools/config/benchmark.yaml), KITTI 07, 03, 04, 06, 10 and 01
(highway) x 3 seeds, 500 particles, defaults otherwise. Means over the 18 runs of each scenario:

| scenario | method | RMSE [m] | max [m] | NEES (3) | 95 % coverage | likelihood evaluations | CPU / CUDA [ms] |
|---|---|---|---|---|---|---|---|
| nominal | importance | 0.160 | 0.73 | diverges | 0.92 | 1 | 1.9 / 1.1 |
| | annealing (5 layers) | 0.081 | 0.28 | 0.44 | 1.00 | 6 | 5.6 / 1.8 |
| | **tempered SMC** | **0.071** | 0.34 | **2.79** | **0.95** | 8.4 | 7.1 / 2.3 |
| | tempered SMC, `scan_max_effective_points: 10` | 0.108 | 0.36 | 0.50 | 1.00 | 5.0 | |
| | tempered SMC, `max_stages: 5` | 0.089 | 0.38 | 59 | 0.75 | 6 | |
| 3 m / 8.6 deg off | annealing | 0.081 | 0.30 | 0.45 | 1.00 | 6 | |
| | tempered SMC | 0.073 | 0.40 | 3.64 | 0.95 | 8.4 | |
| kidnapped (frame 151, 5 m) | importance | 30.7 (4 / 18 wrong mode) | | | | | |
| | annealing | 0.146 | 2.10 | | | | |
| | tempered SMC | 0.117 | 1.73 | | | | |

Translation RMSE per sequence (nominal):

| method | 07 | 03 | 04 | 06 | 10 | 01 |
|---|---|---|---|---|---|---|
| importance | 0.100 | 0.107 | 0.146 | 0.144 | 0.100 | 0.362 |
| annealing | 0.062 | 0.078 | 0.075 | 0.077 | 0.055 | 0.139 |
| tempered SMC | **0.050** | **0.067** | **0.070** | **0.065** | **0.052** | **0.123** |

(timing: KITTI 07 alone, 4 threads; the benchmark ran 14 in parallel.)

- **What annealing gets wrong**: it applies the likelihood once per layer on top of the update and
  moves the particles by a Gaussian that keeps no distribution. On a Gaussian prior and
  observation whose posterior is known, its variance is 20-25 % too small on every axis; tempered
  SMC matches it within 15 % (`test_tempered_smc.cpp`). On KITTI both are *under*confident (NEES
  0.4-0.5) because `scan_max_effective_points: 10` softens the likelihood that much, which a
  single update needs to avoid collapsing (importance with 100 on KITTI 07: 16 % coverage, 0.144
  instead of 0.092 m).
- **Tempered SMC** (`TemperedSmcUpdate`) multiplies the weights by L^(tau db) per stage, the
  increments db chosen by bisection to keep the conditional ESS (Zhou et al., 2016) at N/2, until
  b = 1, so the likelihood is applied exactly once and needs no softening: with
  `scan_max_effective_points: 100` the covariance is consistent and the error 12 % lower than
  annealing, on every sequence. After each stage the particles are resampled if the ESS is below
  half and moved by a random walk Metropolis step invariant for p(x | ancestor, odometry)
  L(x)^(tau b) (resample-move). The predicted distribution is a mixture over the previous
  particles; on the extended space (ancestor, pose) the motion prior of a particle is the density
  of its own prediction, which `TransitionPrior` linearizes: the noise-free prediction of the
  ancestor (anchor) and the covariance of the motion model noise propagated through every step
  (exact to first order for one twist per scan, checked against Monte Carlo predictions).
- **Proposals**: per cluster of particles (`ParticleClustering`), the weighted covariance before
  resampling with clamped eigenvalues (1 rad counted as 5 m), scaled by 2.38 / sqrt(6) and adapted
  to a 0.3 acceptance rate (measured 0.29-0.31).
- **Budget**: `max_stages` caps the cost; capped at annealing's 6 evaluations, the forced last
  stage degenerates (NEES 59). A lower conditional ESS target does better at that budget (next
  section).
- **Delayed scans**: moves break the genealogy that reweighting a late measurement relies on, so
  states updated with moves (annealing too) make late measurements replay instead.

### Gradient-based moves and the default

`LangevinMove` proposes delta = h/2 M g + sqrt(h) M^(1/2) z in the frame of each particle (g the
gradient of the log target, motion prior included) and accepts with the Metropolis-Hastings ratio
of the reverse proposal. The gradients come from the nearest map points the likelihood field
already finds (`PointCloudMap::nearestPoint()`; exact within their Voronoi cells, checked by
finite differences), on the CPU and on the GPU (17 sufficient statistics per particle reduced in
the kernel), with the Gauss-Newton curvature for the preconditioner. The derivatives of accepted
proposals are kept, so a step costs one evaluation like the random walk. Same benchmark as above:

| method | conditional ESS | nominal RMSE [m] | 3 m off | kidnapped | NEES | evaluations | KITTI 07 CPU / CUDA [ms] |
|---|---|---|---|---|---|---|---|
| annealing (former default) | - | 0.081 | 0.081 | 0.146 | 0.44 | 6 | 5.7 / 1.8 |
| random walk | 0.5 | **0.071** | **0.073** | 0.117 | 2.79 | 8.4 | 7.1 / 2.3 |
| **random walk (default)** | 0.15 | 0.075 | 0.075 | **0.104** | 3.38 | **5.3** | **4.8 / 1.8** |
| MALA | 0.3 | 0.224 | 0.232 | 0.386 | diverges | 5.8 | |
| preconditioned MALA (cluster covariance) | 0.3 | 0.072 | 0.075 | 0.124 | 3.00 | 6.6 | 8.3 / - |
| Gauss-Newton | 0.3 | **0.071** | **0.071** | 0.173 | 3.03 | 6.6 | |
| Gauss-Newton | 0.15 | 0.072 | 0.073 | 0.257 | 2.98 | 5.2 | 6.8 / 2.3 |

- At the cost of annealing or less, tempered SMC with random walk moves and a conditional ESS
  of 0.15 (fewer, larger stages) is more accurate in every scenario and its covariance is
  consistent: it is the node default now (`scan_tempered_smc`, `scan_max_effective_points: 100`).
- Gauss-Newton moves are the most accurate near the truth (acceptance 0.47-0.52) but recover
  worse from a wrong pose: the local curvature pulls a particle into the nearest basin, which is
  what a far-off particle should leave. Preconditioned MALA is in between. Isotropic MALA does not
  mix in 6 DoF (position and rotation scales differ; acceptance 0.14).
- All kernels keep the analytic posterior of the Gaussian test (`GradientKernelsTargetThePosterior`).

### Tempering

The points of a scan are not independent (they see the same surfaces, with the same map and
calibration errors), but the likelihood multiplies them as if they were. `scan_max_effective_points`
lets a scan count as at most that many points (the log-likelihood is scaled by
min(1, limit / points)); the per-point likelihood used by recovery is not scaled. Default
configuration, 6 seeds:

| `scan_max_effective_points` | translation RMSE mean (range) [m] | yaw RMSE mean [deg] | max [m] |
|---|---|---|---|
| 0 (no limit, 200 points) | 0.165 (0.157-0.169) | 0.20 | 0.53 |
| 50 | 0.136 (0.132-0.140) | 0.17 | 0.46 |
| 25 | 0.120 (0.113-0.122) | 0.15 | 0.47 |
| **10** (default) | **0.102 (0.092-0.108)** | **0.12** | 0.41 |
| 5 | 0.101 (0.097-0.106) | 0.12 | 0.34 |
| 2 | 0.123 (0.119-0.126) | 0.12 | 0.41 |

The gain holds for every configuration tried (3 seeds): CUDA with 10000 particles 0.088 ->
0.069 m, the vehicle model 0.133 -> 0.10 m, NDT 0.18 -> 0.11 m. It also decouples the sharpness
from `scan_sample_num` (1000 points without a limit: 0.165 m). Initialized 1.5 m and 6 deg off, the
filter converges as fast as without the limit (2-9 frames). The price is slower convergence on
repeated identical scans: from a static scan, global localization in the synthetic test needs
~100 updates instead of 40 (but no seed locks onto the mirrored pose any more).

### Delayed measurements

`late_every=3 late_by=2` delivers every third scan after the next two (0.4 s late):

| | updates replayed | update mean / p95 / max [ms] | final state |
|---|---|---|---|
| in time order | 0 | 1.8 / 2.1 / 2.4 | - |
| every third scan late | 364 | 2.4 / 3.7 / 5.3 | identical |
| CUDA, 10000 particles, every third scan late | 364 | 2.6 / 3.7 / 4.4 | identical |

The estimates published while a scan is missing cannot use it, so the error of the published
poses is higher; once it arrives, the history is the one of time-ordered arrival, bit for bit.

Replaying versus reweighting late scans (`late_fusion=replay|reweight`), against dropping them
(`max_delay=0`); translation RMSE of the published estimates, mean (range) over seeds 1-6:

| every third scan late by | dropped | replayed | reweighted |
|---|---|---|---|
| 0.4 s (`late_by=2`) | 0.133 (0.128-0.140) | 0.126 (0.122-0.128) | 0.138 (0.127-0.149) |
| 1.4 s (`late_by=5`) | 0.133 (0.128-0.140) | 0.131 (0.124-0.135) | 0.138 (0.129-0.146) |
| 2.8 s (`late_by=10`, `max_delay=4`) | 0.133 (0.128-0.140) | 0.133 (0.123-0.138) | 0.135 (0.124-0.145) |

(0.102 in time order.) Update time per delivered scan, mean / p95 / max [ms], seed 1:

| every third scan late by | replayed | reweighted |
|---|---|---|
| 0.4 s | 2.4 / 3.6 / 4.4 | 2.1 / 2.6 / 3.0 |
| 2.8 s | 4.6 / 10.8 / 14.0 | 2.1 / 2.7 / 3.9 |

Reweighting keeps the cost of a late scan at one update whatever its delay, but on KITTI it was
no more accurate than dropping the scan, presumably because the newer scans already carry its
information. The exact replay is therefore the default. Reweighting pays off for a slow sensor
whose information the others lack, behind a fast one: in the tests, an absolute pose sensor 1.5 s
late behind 10 Hz updates, reweighted, is nearly as accurate as in time order (0.15 vs 0.14 m over
ten seeds, 0.22 m without the sensor), with no replay.

### Scan likelihoods

Same protocol and defaults (500 particles), 3 seeds:

| `point_cloud_likelihood.type` | translation RMSE [m] | yaw RMSE [deg] | max [m] | update mean / p95 [ms] | map memory |
|---|---|---|---|---|---|
| `likelihood_field` (distance field 0.2 m, default) | **0.092-0.105** | **0.12** | 0.41 | **1.8 / 2.1** | 227 MB |
| `ndt` (1 m, 7 voxels, default) | 0.111-0.114 | 0.13-0.14 | 0.34 | 2.7 / 3.0 | **20 MB** |
| `ndt` (1 m, `search_neighbors: false`) | 0.141-0.156 | 0.17-0.18 | 0.60 | 2.0 / 2.2 | 15 MB |
| `gicp` (1 m, 7 voxels, 10 neighbors) | 0.114-0.118 | 0.12-0.13 | 0.33 | 4.0 / 4.5 | 20 MB |

NDT trades ~10 % of accuracy for a 10x smaller map, whose size depends on the mapped surface
rather than on the volume around it. Other NDT settings were less accurate before tempering:
0.5 m voxels 0.183, 2 m 0.193, `sigma` 0.1 0.188 and 0.3 0.185 vs 0.177 (seed 1). Scoring with
the face-neighboring voxels widens the basin beyond one voxel and removes the jumps at voxel
boundaries; precomputing the candidate voxels of each cell keeps it at one hash lookup per point.

GICP did not improve on NDT here (measured before tempering; with it both are within 0.01 m).
Adding the scan covariance (from the 10 nearest points of the
0.5 m filtered scan) widens each pair of distributions and blurs the score; Segal-style
plane-ized covariances, 0.5 m voxels and `sigma` 0.05-0.1 all stayed at 0.185-0.199 (seed 1),
and with zero scan covariances it reduces to NDT exactly (0.177). It may pay off with denser
scans or finer maps, which this benchmark does not cover.

```bash
./kitti_evaluation <kitti>/dataset 07 likelihood=ndt
```

### Motion models

Same protocol, but with odometry like a car provides it (forward speed and yaw rate only,
`odometry=wheel`) and the base frame at the rear axle center (`base_x=0.86 base_pitch=0.0143`,
estimated from the ground truth as the point with the least lateral velocity and the direction of
travel), 3 seeds:

| `motion_model.type` | translation RMSE [m] | yaw RMSE [deg] | max [m] |
|---|---|---|---|
| `twist` | 0.102-0.111 | 0.12-0.13 | 0.41 |
| `vehicle` | **0.093-0.110** | **0.09-0.10** | 0.46 |
| `vehicle`, base frame not aligned (`base_pitch=0`) | 0.092-0.110 | 0.09-0.10 | 0.40 |

Before tempering the vehicle model also had 25 % lower translation error (0.115-0.122 vs
0.158-0.176 m); with it, the gain is in yaw (25 %).

The velodyne of KITTI is pitched by 0.84 deg against the direction of travel. A vehicle model
without vertical slip noise moved the particles along that tilted axis, sank by 0.1 m/s and
diverged; `vertical_noise_scale` makes it robust to such a misalignment (last row). During the
5 s stop of the sequence both models are equally accurate. Freezing the particles while stopped
was tried and dropped: it gave no gain on KITTI, but a car initialized while parked then stayed
at the error of its best initial particle (the first scan collapses the cloud), so the noise keeps
a floor of `min_noise_speed`.

```bash
./kitti_evaluation <kitti>/dataset 07 motion=vehicle odometry=wheel base_x=0.86 base_pitch=0.0143
```

Caveats: map and localized scans come from the same drive (not a test of robustness to map
changes), the KITTI ground truth itself has errors in the order of 0.1 m, and scans are not
deskewed. Reproduce:

```bash
# sequence 07 from https://www.cvlibs.net/datasets/kitti/eval_odometry.php (velodyne, calib, poses)
./kitti_evaluation <kitti>/dataset 07 map=distance_field resolution=0.2 seed=1 output=trajectory.csv
```

`likelihood_benchmark` measures the likelihood evaluation alone (synthetic 40 m map):

| particles x points | `kdtree` 1 / 4 threads [ms] | `distance_field` 0.1 m 1 / 4 threads [ms] | CUDA [ms] |
|---|---|---|---|
| 500 x 100 | 77 / 22 | 2.4 / 0.6 | 0.05 |
| 2000 x 300 | 971 / 256 | 23 / 6.4 | 0.09 |
| 10000 x 300 | 4709 / 1290 | 120 / 29 | 0.23 |
| 10000 x 2000 | - | - / 196 | 0.72 |

CUDA includes the upload of poses and points and the download of the sums. The GPU computes in
single precision; sums agree with the CPU to rounding except for rare points on voxel boundaries
(see `test_cuda_distance_field_map.cpp`).

## Limitations

- Memory of the distance field grows with the mapped area (~0.3 MB per meter of route at 0.2 m);
  very large maps need `ndt` (~0.03 MB per meter) or tiling (not implemented).
- With KLD sampling the particle count shrinks to `min_particle_num`, which costs accuracy
  (see Evaluation); raise it if accuracy matters more than computation.
- Global localization assumes a ground vehicle (single ground level) and can lock onto a wrong
  mode in (nearly) symmetric environments.
- The CUDA backend copies the whole distance field to the GPU (227 MB for KITTI 07 at 0.2 m).
- PCL works in single precision: shift maps in large coordinates (e.g. UTM) near the origin.

## Roadmap

- Prediction, resampling and estimation on the GPU, to keep particles resident there (the CPU
  side dominates beyond ~50000 particles)
- Map tiling for large areas; evaluation on more datasets (map changes, other sensors)

## References

- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* — http://www.probabilistic-robotics.org/
- D. Fox, "Adapting the sample size in particle filters through KLD-sampling", IJRR 2003
- F. L. Markley et al., "Averaging quaternions", JGCD 2007
- P. F. Felzenszwalb, D. P. Huttenlocher, "Distance Transforms of Sampled Functions", ToC 2012
- E. Takeuchi, K. Ohno, S. Tadokoro, "Modularized retroactive particle filter" (in Japanese),
  ROBOMECH 2010, 1A2-E15 — https://doi.org/10.1299/jsmermd.2010._1A2-E15_1
- S. Maeyama, A. Ohya, S. Yuta, "Non-stop outdoor navigation of a mobile robot: retroactive
  positioning data fusion with a time consuming sensor system", IROS 1995
- R. Ueda et al., "Expansion resetting for recovery from fatal error in Monte Carlo localization", IROS 2004
- M. Magnusson, "The three-dimensional normal-distributions transform", 2009
- Effective sample size — http://www.cns.nyu.edu/~eorhan/notes/particle-filtering.pdf
- Avoiding numerical underflow — http://www.maths.lu.se/fileadmin/maths/forskning_research/InferPartObsProcess/particlemethods.pdf
- Japanese: http://lab.cntl.kyutech.ac.jp/~nishida/lecture/psc/no11.pdf ,
  https://gihyo.jp/book/2018/978-4-7741-9646-6 , https://qiita.com/MoriKen/items/dfb6eb168649873589f0
