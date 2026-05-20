# FAST_LIO2 Mapping and Localization

A LiDAR-Inertial SLAM package based on FAST-LIO2, integrating a **DOP (Dilution of Precision)-based scan matching confidence evaluation** method for robust odometry and pose graph optimization. Two operation modes are supported: **Mapping** and **Localization**.

> **Base**: [FAST_LIO_ROS2 (Ericsii)](https://github.com/Ericsii/FAST_LIO_ROS2)  

---

## Key Features

- **Dual Operation Modes**: Separate Mapping mode (map building) and Localization mode (pose estimation against a pre-built map)
- **DOP-Based Scan Matching Confidence Evaluation**: Applies the DOP concept from GNSS to LiDAR scan matching, dynamically adjusting measurement weights
  - **Scan DOP**: Evaluates the geometric distribution of the input scan to dynamically adjust the voxel filter size
  - **Matching DOP**: Evaluates the geometric distribution of matched points to adaptively set the EKF measurement covariance
- **Pose Graph Optimization Integration**: Automatically integrates with the `pose_graph_optimization` node in Mapping mode, with DOP-based loop closure reliability filtering
- **Octomap Integration**: Integrates with `octomap_server` in Localization mode for real-time occupancy grid map generation
- **Multi-LiDAR Support**: Livox (Avia, MID360, MID70), Velodyne, Ouster (32/64/128), Hesai32
- **Performance Statistics**: Automatically prints processing time statistics (point matching, KD-Tree, EKF update, etc.) on shutdown

---

## Algorithm Overview

### FAST-LIO2 Core Pipeline

FAST-LIO2 estimates robot pose using an Iterated Error State Kalman Filter (IESKF) that tightly couples IMU data with LiDAR point clouds.

1. **IMU Forward Propagation**: Predicts robot state using IMU measurements
2. **LiDAR Motion Undistortion**: Corrects scan distortion using IMU integration results
3. **Point Matching**: Scan-to-map point matching using ikd-Tree
4. **IESKF Update**: Updates state and covariance using matching residuals as observations

### DOP-Based Scan Matching Confidence Evaluation

The DOP concept from GNSS — which quantifies the geometric arrangement between satellites and the receiver — is applied to LiDAR scan matching.

**DOP Computation**

Distance from the sensor origin to each matched point $i$:

$$r_i = \sqrt{(x_i - x_s)^2 + (y_i - y_s)^2 + (z_i - z_s)^2}$$

Unit vector matrix:

$$A = \begin{bmatrix} \frac{x_1 - x_s}{r_1} & \frac{y_1 - y_s}{r_1} & \frac{z_1 - z_s}{r_1} \\ \vdots & \vdots & \vdots \\ \frac{x_n - x_s}{r_n} & \frac{y_n - y_s}{r_n} & \frac{z_n - z_s}{r_n} \end{bmatrix}$$

Covariance matrix and DOP:

$$Q = (A^T A)^{-1}, \quad \text{DOP} = \sqrt{\text{tr}(Q)}$$

**Scaling via Tukey Loss Function**

$$s(\sigma) = \begin{cases}
c^2\left(1 - \left[1 - \left(\frac{\sigma}{c}\right)^2\right]^3\right) & \text{if } \sigma \leq c \\
c^2 & \text{otherwise}
\end{cases}$$

The computed $s(\sigma)$ is multiplied with the LiDAR measurement covariance to dynamically adjust the measurement weight:

$$R = s(\rho) \cdot \text{diag}(\sigma_L^2)$$

**Two-Stage DOP Usage**

| Stage | Input | Usage |
|-------|-------|-------|
| Scan DOP | Full input scan point cloud | Dynamically adjusts voxel filter size (reduces filter size when scan quality is poor) |
| Matching DOP | Matched points used in IESKF observation | Dynamically sets EKF measurement covariance (`lidar_meas_cov`) |

---

## Prerequisites

### 1. Ubuntu & ROS2

- **Ubuntu >= 20.04** (Recommended: Ubuntu 22.04)
- **ROS >= Foxy** (Recommended: ROS Humble)
  - [ROS Humble Installation](https://docs.ros.org/en/humble/Installation.html)

### 2. PCL & Eigen

```bash
sudo apt install libpcl-dev libeigen3-dev
```

- PCL >= 1.8
- Eigen >= 3.3.4

### 3. livox_ros_driver2

Required when using Livox series LiDAR:

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble
```

Add to `~/.bashrc`:

```bash
source ~/ws_livox/install/setup.bash
```

### 4. Dependencies (Mapping Mode)

Mapping mode integrates with the `pose_graph_optimization` package:

```bash
cd <ros2_ws>/src
git clone https://github.com/Kimkyuwon/Pose_Graph_Optimization.git
```

### 5. Dependencies (Localization Mode)

Localization mode integrates with `octomap_server`:

```bash
sudo apt install ros-humble-octomap-server
```

---

## Build

```bash
cd <ros2_ws>/src
git clone https://github.com/Kimkyuwon/fast_lio2_mapping_and_localization.git --recursive fast_lio
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select fast_lio
source install/setup.bash
```

> **Note**: When using Livox LiDAR, make sure `livox_ros_driver2` is sourced before building.

---

## Usage

### Mapping Mode

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mapping_config.yaml
```

Change `config_file` according to your LiDAR:

| LiDAR | Config File |
|-------|-------------|
| Hesai Pandar 32 | `mapping_config.yaml` |
| Livox Avia | `avia.yaml` |
| Livox MID360 | `mid360.yaml` |
| Livox MID70 | `mid70.yaml` |
| Velodyne | `velodyne.yaml` |
| Ouster 32 | `ouster32.yaml` |
| Ouster 64 | `ouster64.yaml` |
| Ouster 128 | `ouster128.yaml` |

**Launch the LiDAR driver separately when using real hardware**:

```bash
# MID360 example
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Localization Mode

```bash
ros2 launch fast_lio localization.launch.py config_file:=localization_config.yaml
```

> Set `map_file_path` in `localization_config.yaml` to the path of a pre-built `.pcd` map file.

---

## Performance (DOP Method Results)

The DOP-based scan matching confidence evaluation was validated on the following datasets.

### Hilti SLAM Challenge 2022 (ATE, m)

| Method | exp-06 RMSE | exp-14 RMSE | exp-16 RMSE | exp-18 RMSE |
|--------|------------|------------|------------|------------|
| FAST-LIO2 | 0.039 | 0.058 | div. | 1.452 |
| Faster-LIO | 0.070 | 0.090 | div. | 4.000 |
| **Proposed** | **0.045** | 0.077 | **0.636** | **0.156** |

> exp-16 and exp-18 include narrow staircase segments where existing methods diverge. The proposed DOP method maintains stable estimation throughout.

### VBR SLAM Dataset (ATE, m)

| Method | Colosseum RMSE | Diag RMSE |
|--------|---------------|----------|
| Faster-LIO | 2.730 | 0.217 |
| Faster-LIO + PGO | 0.386 | 0.179 |
| **Proposed (DOP + PGO)** | **0.250** | **0.165** |

---

## Related Papers

```bibtex
@article{xu2022fastlio2,
  title     = {FAST-LIO2: Fast Direct LiDAR-Inertial Odometry},
  author    = {Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal   = {IEEE Transactions on Robotics},
  volume    = {38},
  number    = {4},
  pages     = {2053--2073},
  year      = {2022}
}
```

```bibtex
@article{kim2025dop,
  title   = {Scan Matching Confidence Evaluation for Robust LiDAR Odometry and Pose Graph Optimization},
  author  = {Kim, Kyu-Won},
  journal = {Journal of Institute of Control, Robotics and Systems},
  volume  = {31},
  number  = {11},
  pages   = {1299--1306},
  year    = {2025},
  doi     = {10.5302/J.ICROS.2025.25.0088}
}
```

---

## Related Projects

**SLAM:**
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree): State-of-the-art dynamic KD-Tree for 3D kNN search
- [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): Re-localization integration for FAST-LIO
- [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init): Robust real-time LiDAR-IMU extrinsic initialization

**Control and Planning:**
- [IKFoM](https://github.com/hku-mars/IKFoM): Toolbox for fast on-manifold Kalman filter

---

## Acknowledgments

- [FAST-LIO2 (HKU-MARS)](https://github.com/hku-mars/FAST_LIO): Core LiDAR-Inertial Odometry algorithm
- [FAST_LIO_ROS2 (Ericsii)](https://github.com/Ericsii/FAST_LIO_ROS2): ROS2 porting base code
- [LOAM](https://github.com/laboshinl/loam_velodyne): Original LiDAR Odometry algorithm (Ji Zhang, Carnegie Mellon University)
- [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)

---

## License

BSD License
