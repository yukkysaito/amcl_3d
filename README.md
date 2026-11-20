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

- **Prediction**: odometry twists (optionally with IMU angular velocity) are buffered and integrated
  on SE(3) up to each scan stamp; white velocity noise independent of the odometry rate, with
  - `twist`: generic 6-DoF motion with isotropic noise, or
  - `vehicle`: car-like motion (forward speed and yaw rate, base frame at the rear axle) with
    anisotropic noise growing with the speed; 25-30 % lower translation and 35 % lower yaw
    error than `twist` on KITTI with wheel-like odometry (see Evaluation)
- **Correction** by a scan likelihood (`point_cloud_likelihood.type`), robust to points not in
  the map by clipping:
  - `likelihood_field` (default): distance from each point to the nearest map point, on
    - `distance_field` (default): precomputed sparse field of nearest map points, ~12x faster
      updates than `kdtree` with the same accuracy on KITTI (see Evaluation), or
    - `kdtree`: exact nearest neighbor search
  - `ndt`: Mahalanobis distance to the normal distributions of the map points in 1 m voxels;
    a 10x smaller map than the distance field, at 15 % more error on KITTI
  - particles are evaluated in parallel on `thread_num` CPU threads (OpenMP), or on a CUDA
    GPU (`device: cuda`, likelihood field), which affords 10-100x more particles
- **Other sensors** plug in as a `MeasurementModel` fused by `Localizer::update()` (see Design)
- **Scan preprocessing**: range / height crop, voxel grid, random subsampling
- **Resampling**: systematic or KLD sampling, triggered by the effective sample size
- **Recovery**: augmented MCL, expansion resetting, global localization
- **Refinement**: optional NDT scan matching of the estimate (with optional feedback)
- Outputs pose with covariance (quaternion averaging) and `map` -> `odom`

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
 ├─ Resampler                  SystematicResampler, KldResampler
 ├─ ScanFilter
 └─ ScanMatcher                NdtScanMatcher
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
- **A new scan likelihood** (e.g. GICP) implements `PointCloudLikelihood`, which owns the map
  representation it needs (distance field, voxel Gaussians, ...), and is added to
  `createPointCloudLikelihood()`.
- **A new map backend** for the likelihood field implements `PointCloudMap` and is added to
  `createPointCloudMap()`; a new motion model implements `MotionModel` and is added to
  `createMotionModel()`.

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
- defaults otherwise: 500 particles, 200 scan points, systematic resampling

| configuration | translation RMSE [m] | yaw RMSE [deg] | max [m] | update mean / p95 [ms] |
|---|---|---|---|---|
| `kdtree` | 0.158-0.170 | 0.19-0.20 | 0.57 | 24.0 / 27.9 |
| `distance_field` 0.2 m (default) | 0.153-0.158 | 0.20-0.21 | 0.49 | **1.8 / 2.2** |
| `distance_field` 0.1 m | 0.161-0.167 | 0.19-0.21 | 0.57 | 2.4 / 2.9 |
| `distance_field` 0.2 m + NDT | 0.139-0.141 | 0.18-0.19 | 0.47 | 35.6 / 60.9 |
| `distance_field` 0.2 m + KLD (~100 particles) | 0.221-0.243 | 0.27-0.29 | 0.85 | 1.2 / 1.4 |
| `distance_field` 0.2 m, 10000 particles | 0.089-0.103 | 0.12-0.13 | 0.34 | 15.5 / 18.0 |
| `distance_field` 0.2 m, CUDA | 0.153-0.158 | 0.20-0.21 | 0.49 | 1.0 / 1.2 |
| `distance_field` 0.2 m, CUDA, 10000 particles | 0.088-0.091 | 0.12-0.13 | 0.34 | 2.0 / 2.4 |
| `distance_field` 0.2 m, CUDA, 50000 particles | **0.067-0.085** | **0.09-0.10** | 0.30 | 5.7 / 7.4 |

Ranges over 3 seeds; timing with `thread_num: 4` on an i9-14900K (the default 500 particles
take 4.1 / 5.0 ms on one thread, 1.5 / 1.8 ms on 8). The results do not depend on the thread
count. No frame was lost (error > 1 m). The distance field takes 4 s to
build and 227 MB (1.2 GB at 0.1 m) for this route.

More particles pay off: 10000 particles nearly halve the error of the default 500, and 50000 cut
it to 0.07 m, which the GPU backend (RTX 5080) keeps within 6 ms per update. More scan points do
not help (10000 particles x 1000 points: 0.093-0.097 m). With 50000 particles the likelihood takes
under 1 ms; prediction runs in parallel on `thread_num` threads, and the rest is resampling and the
estimate on one thread.

Using every hardware thread (`thread_num: 0`, 32 threads) was slower than 4-16 threads
(6.2 ms): idle OpenMP threads spin on the cores the rest of the update runs on. Setting
`OMP_WAIT_POLICY=passive` avoids that.

### Scan likelihoods

Same protocol and defaults (500 particles), 3 seeds:

| `point_cloud_likelihood.type` | translation RMSE [m] | yaw RMSE [deg] | max [m] | update mean / p95 [ms] | map memory |
|---|---|---|---|---|---|
| `likelihood_field` (distance field 0.2 m, default) | **0.153-0.158** | **0.20-0.21** | 0.49 | **1.8 / 2.2** | 227 MB |
| `ndt` (1 m, 7 voxels, default) | 0.177-0.181 | 0.21-0.22 | 0.50 | 2.7 / 3.0 | **20 MB** |
| `ndt` (1 m, `search_neighbors: false`) | 0.199-0.202 | 0.23-0.24 | 0.66 | 1.7 / 2.1 | 15 MB |

NDT trades 15 % of accuracy for a 10x smaller map, whose size depends on the mapped surface rather
than on the volume around it. Other NDT settings were less accurate: 0.5 m voxels 0.183, 2 m
0.193, `sigma` 0.1 0.188 and 0.3 0.185 (seed 1). Scoring with the face-neighboring voxels widens
the basin beyond one voxel and removes the jumps at voxel boundaries; precomputing the candidate
voxels of each cell keeps it at one hash lookup per point.

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
| `twist` | 0.158-0.176 | 0.20-0.22 | 0.59 |
| `vehicle` | **0.115-0.122** | **0.13-0.14** | 0.42 |
| `vehicle`, base frame not aligned (`base_pitch=0`) | 0.129-0.134 | 0.13-0.14 | 0.58 |

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
- R. Ueda et al., "Expansion resetting for recovery from fatal error in Monte Carlo localization", IROS 2004
- M. Magnusson, "The three-dimensional normal-distributions transform", 2009
- Effective sample size — http://www.cns.nyu.edu/~eorhan/notes/particle-filtering.pdf
- Avoiding numerical underflow — http://www.maths.lu.se/fileadmin/maths/forskning_research/InferPartObsProcess/particlemethods.pdf
- Japanese: http://lab.cntl.kyutech.ac.jp/~nishida/lecture/psc/no11.pdf ,
  https://gihyo.jp/book/2018/978-4-7741-9646-6 , https://qiita.com/MoriKen/items/dfb6eb168649873589f0
