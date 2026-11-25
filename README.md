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
  arrived in time (see Design)
- **Prediction**: odometry twists (optionally with IMU angular velocity) are buffered and integrated
  on SE(3) up to each measurement stamp; white velocity noise independent of the odometry rate,
  with
  - `twist`: generic 6-DoF motion with isotropic noise, or
  - `vehicle`: car-like motion (forward speed and yaw rate, base frame at the rear axle) with
    anisotropic noise growing with the speed; 25 % lower yaw error than `twist` on KITTI with
    wheel-like odometry ([evaluation](docs/evaluation.md#motion-models))
- **Correction** by a scan likelihood (`point_cloud_likelihood.type`), robust to points not in
  the map by clipping:
  - `likelihood_field` (default): distance from each point to the nearest map point, on
    - `distance_field` (default): precomputed sparse field of nearest map points, ~12x faster
      updates than `kdtree` with the same accuracy on KITTI, or
    - `kdtree`: exact nearest neighbor search
  - `ndt`: Mahalanobis distance to the normal distributions of the map points in 1 m voxels;
    a 10x smaller map than the distance field, at ~10 % more error on KITTI
  - tempered: a scan counts as at most `scan_max_effective_points` independent points, since
    its points are strongly correlated
  - particles are evaluated in parallel on `thread_num` CPU threads (OpenMP), or on a CUDA
    GPU (`device: cuda`, likelihood field), which affords 10-100x more particles
- **Other sensors** plug in as a `MeasurementModel` fused by `Localizer::update()` (see Design)
- **Scan preprocessing**: range / height crop, voxel grid, random subsampling
- **Resampling**: systematic or KLD sampling, triggered by the effective sample size
- **Recovery**: augmented MCL, expansion resetting (statistics per sensor), global localization
- **Tempered SMC** (default): each scan enters in adaptive stages with MCMC moves in between
  (`scan_tempered_smc`; random walk or Gauss-Newton moves, or for ground vehicles a planar
  random walk with the vertical axes drawn from their conditional Gauss-Newton model), which
  gather the cloud where the scan fits rather than where the motion model scattered it, while
  keeping the posterior: the particle covariance matches the error (NEES ~3); optionally coarse to
  fine (early stages with a quarter of the scan points)
- **Refinement**: optional GICP scan matching of the published estimate (cm-level)
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
 │           └─ NdtLikelihood      on GaussianVoxelMap
 ├─ MeasurementUpdate          per sensor: ImportanceUpdate, TemperedSmcUpdate
 │   └─ MoveKernel             MCMC moves of tempered SMC: RandomWalkMetropolis,
 │                             GaussNewtonMove (log-likelihood gradients and Gauss-Newton
 │                             curvature from LikelihoodField on CPU and CUDA, PoseMeasurementModel)
 ├─ Resampler                  SystematicResampler, KldResampler
 ├─ ScanFilter
 └─ ScanMatcher                GicpScanMatcher
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
  (`createMeasurementModel()`), so that per-scan features (e.g. local covariances) are computed
  once per scan.
- **A new map backend** for the likelihood field implements `PointCloudMap` and is added to
  `createPointCloudMap()`; a new motion model implements `MotionModel` and is added to
  `createMotionModel()`.

Per sensor, `SensorParams` sets how many independent observations one update may count as
(tempering) and whether the sensor drives recovery. Augmented MCL and expansion resetting keep
their statistics per recovery sensor: likelihoods of different sensors are not comparable.

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

Replaying costs delay x update rate updates, e.g. 30 scan updates for a sensor 3 s late. A
retroactive particle filter (reweighting the descendants of the particles at the measurement time)
was evaluated and removed: it cannot follow particles moved by MCMC, so with tempered SMC it always
fell back to replay ([evaluation](docs/evaluation.md#removed-options)).

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

KITTI odometry sequences 07, 03, 04, 06, 10 and 01 (highway), 3 seeds each, 500 particles, the
node defaults (tempered SMC with random walk moves): the map from even frames registered with the
ground truth, odd frames localized with odometry derived from the ground truth with a 2 % scale
error, a yaw rate bias and noise. Means over the 18 runs:

| scenario | translation RMSE [m] | NEES (x, y, yaw; 3 if consistent) | likelihood evaluations / scan | KITTI 07 update CPU / CUDA [ms] |
|---|---|---|---|---|
| initial guess 0.5 m / 1.7 deg off | 0.075 | 3.38 | 5.3 | 4.8 / 1.8 |
| initial guess 3 m / 8.6 deg off | 0.075 | | | |
| kidnapped (5 m, expansion resetting) | 0.104 | | | |

Options measured on the same benchmark:

| configuration | nominal / 3 m off / kidnapped [m] | KITTI 07 CPU / CUDA [ms] |
|---|---|---|
| default | 0.075 / 0.075 / 0.104 | 4.95 / 1.85 |
| `scan_sample_num: 800`, `coarse_to_fine: true` | 0.058 / 0.058 / 0.103 | 9.7 / 2.04 |
| the same, `kernel: vertical_gauss_newton` (ground vehicles), 200 particles | 0.058 / 0.059 / 0.100 | 6.7 / 1.80 |

With 800 points, coarse to fine and GICP refinement of the published pose
([`kitti_accurate.yaml`](amcl_3d_core/tools/config/kitti_accurate.yaml)), KITTI 07 reaches
0.037-0.039 m.
[docs/evaluation.md](docs/evaluation.md) holds every measurement behind the defaults: map
representations, particle counts, tempering, tempered SMC against the annealed particle filter,
move kernels, delayed measurements, scan likelihoods, motion models, and the options removed
because they lost. Reproduce with
[`kitti_benchmark.py`](amcl_3d_core/tools/kitti_benchmark.py) and
[`benchmark.yaml`](amcl_3d_core/tools/config/benchmark.yaml):

```bash
# sequences from https://www.cvlibs.net/datasets/kitti/eval_odometry.php (velodyne, calib, poses)
./kitti_evaluation <kitti>/dataset 07 update=tsmc effective=100 cess=0.15 seed=1
```

## Limitations

- Memory of the distance field grows with the mapped area (~0.3 MB per meter of route at 0.2 m);
  very large maps need `ndt` (~0.03 MB per meter) or tiling (not implemented).
- With KLD sampling the particle count shrinks to `min_particle_num`, which costs accuracy
  ([evaluation](docs/evaluation.md)); raise it if accuracy matters more than computation.
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
