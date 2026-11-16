# amcl_3d

3D Monte Carlo localization (particle filter) of a vehicle in a point cloud map with a LiDAR, for ROS 2.

## Packages

| package | description |
|---|---|
| [`amcl_3d_core`](amcl_3d_core) | ROS independent C++17 library (Eigen, PCL). Particle filter, motion / measurement models, resamplers. |
| [`amcl_3d`](amcl_3d) | ROS 2 node (`rclcpp_components`) wrapping the library. |

## Build

Tested with ROS 2 Humble.

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
| `points` | `sensor_msgs/PointCloud2` | LiDAR scan, any frame with a TF to the base frame |
| `pose` (out) | `geometry_msgs/PoseWithCovarianceStamped` | estimate in the global frame |
| `particles` (out) | `geometry_msgs/PoseArray` | |
| `tf` (out) | | `map` -> `odom` |

Parameters are documented in [`amcl_3d/config/amcl_3d.param.yaml`](amcl_3d/config/amcl_3d.param.yaml).

## Design

```
Localizer                      time handling (buffered twists), resampling policy, augmented MCL
 └─ ParticleFilter             initialize / predict / update / resample / estimate
     ├─ ParticleSet            structure of arrays + log weights (vectorization / GPU friendly)
     ├─ MotionModel            TwistMotionModel: SE(3) integration of the body twist
     ├─ MeasurementModel       PointCloudMeasurementModel (scan vs PointCloudMap), PoseMeasurementModel
     └─ Resampler              SystematicResampler, KldResampler
```

`MotionModel`, `MeasurementModel`, `Resampler` and `PointCloudMap` are the extension points.

- Odometry twists are buffered and integrated piecewise up to each scan stamp, so delayed scans are
  corrected at the right time.
- Velocity noise is white noise (std grows with sqrt(dt)), independent of the odometry rate.
- Weights are kept in the log domain; the orientation mean uses quaternion averaging.

## Roadmap

- Likelihood field (precomputed 3D distance map) instead of k-d tree search
- Scan preprocessing (voxel grid, range / height filters)
- Expansion / sensor resetting, global localization
- Scan matching (NDT / GICP) refinement
- CUDA backend for the likelihood computation
- Evaluation on public datasets, CI

## References

- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* — http://www.probabilistic-robotics.org/
- D. Fox, "Adapting the sample size in particle filters through KLD-sampling", IJRR 2003
- F. L. Markley et al., "Averaging quaternions", JGCD 2007
- Effective sample size — http://www.cns.nyu.edu/~eorhan/notes/particle-filtering.pdf
- Avoiding numerical underflow — http://www.maths.lu.se/fileadmin/maths/forskning_research/InferPartObsProcess/particlemethods.pdf
- Japanese: http://lab.cntl.kyutech.ac.jp/~nishida/lecture/psc/no11.pdf ,
  https://gihyo.jp/book/2018/978-4-7741-9646-6 , https://qiita.com/MoriKen/items/dfb6eb168649873589f0
