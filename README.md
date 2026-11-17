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

then set `map.device: cuda`. `CMAKE_CUDA_ARCHITECTURES` defaults to the GPU of the build machine
(`native`); set it explicitly (e.g. `"86;89"`) when building elsewhere. Without CUDA, the node
reports `map.device: cuda` as an error at startup.

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
  on SE(3) up to each scan stamp; white velocity noise independent of the odometry rate
- **Correction**: point-to-map distance likelihood with clipping, on
  - `distance_field` (default): precomputed sparse field of nearest map points, ~12x faster
    updates than `kdtree` with the same accuracy on KITTI (see Evaluation), or
  - `kdtree`: exact nearest neighbor search
  - particles are evaluated in parallel on `map.thread_num` CPU threads (OpenMP), or on a CUDA
    GPU (`map.device: cuda`), which affords 10-100x more particles
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
 ├─ MotionModel                TwistMotionModel
 ├─ MeasurementModel           PointCloudMeasurementModel, PoseMeasurementModel
 │   └─ PointCloudMap          KdTreeMap, DistanceFieldMap, CudaDistanceFieldMap
│                             (batched query over particles x points: OpenMP, or a GPU override)
 ├─ Resampler                  SystematicResampler, KldResampler
 ├─ ScanFilter
 └─ ScanMatcher                NdtScanMatcher
GroundPoseSampler              initial distribution for global localization
```

`MotionModel`, `MeasurementModel`, `PointCloudMap`, `Resampler` and `ScanMatcher` are the
extension points.

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
| `kdtree` | 0.159-0.164 | 0.18-0.20 | 0.56 | 23.6 / 27.4 |
| `distance_field` 0.2 m (default) | 0.154-0.165 | 0.20-0.21 | 0.57 | **1.9 / 2.3** |
| `distance_field` 0.1 m | 0.162-0.165 | 0.19-0.21 | 0.56 | 2.4 / 3.0 |
| `distance_field` 0.2 m + NDT | 0.138-0.145 | 0.17-0.19 | 0.51 | 37.9 / 65.1 |
| `distance_field` 0.2 m + KLD (~100 particles) | 0.226-0.241 | 0.28-0.29 | 1.01 | 1.2 / 1.5 |
| `distance_field` 0.2 m, 10000 particles | 0.089-0.091 | 0.12 | 0.36 | 16.5 / 19.2 |
| `distance_field` 0.2 m, CUDA | 0.155-0.165 | 0.20-0.21 | 0.47 | 1.0 / 1.2 |
| `distance_field` 0.2 m, CUDA, 10000 particles | 0.090-0.092 | 0.12 | 0.31 | 2.7 / 3.1 |
| `distance_field` 0.2 m, CUDA, 50000 particles | **0.068-0.071** | **0.09** | 0.28 | 9.6 / 11.4 |

Ranges over 3 seeds; timing with `map.thread_num: 4` on an i9-14900K (the default 500 particles
take 4.1 / 5.0 ms on one thread, 1.5 / 1.8 ms on 8). The results do not depend on the thread
count. No frame was lost (error > 1 m) except one frame with KLD. The distance field takes 4 s to
build and 227 MB (1.2 GB at 0.1 m) for this route.

More particles pay off: 10000 particles nearly halve the error of the default 500, and 50000 cut
it to 0.07 m, which the GPU backend (RTX 5080) keeps within 10 ms per update. More scan points do
not help (10000 particles x 1000 points: 0.093-0.097 m). Of the 9.6 ms with 50000 particles, the likelihood
takes under 1 ms (extrapolated from `likelihood_benchmark`); the rest is the CPU side (prediction,
resampling, estimate).

Using every hardware thread (`map.thread_num: 0`, 32 threads) was slower than 4-16 threads
(6.2 ms): idle OpenMP threads spin on the cores the rest of the update runs on. Setting
`OMP_WAIT_POLICY=passive` avoids that.

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
  very large maps need tiling (not implemented).
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
