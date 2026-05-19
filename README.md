# FAST_LIO2 Mapping and Localization

FAST-LIO2 기반의 LiDAR-Inertial SLAM 패키지로, **DOP(Dilution of Precision) 기반 스캔 매칭 신뢰도 평가 기법**을 통합하여 강인한 Odometry 및 포즈 그래프 최적화를 지원합니다. 매핑(Mapping)과 로컬라이제이션(Localization) 두 가지 운용 모드를 제공합니다.

> **Base**: [FAST_LIO_ROS2 (Ericsii)](https://github.com/Ericsii/FAST_LIO_ROS2)  
> **Extended by**: Kyu-Won Kim (김규원)

---

## 주요 특징

- **이중 운용 모드**: 매핑 모드(지도 생성)와 로컬라이제이션 모드(사전 지도 기반 위치 추정) 분리 지원
- **DOP 기반 스캔 매칭 신뢰도 평가**: GNSS에서 활용하는 DOP 개념을 LiDAR 스캔 매칭에 적용하여 측정치 가중치를 동적으로 조정
  - **스캔 DOP**: 입력 스캔의 기하학적 분포를 평가하여 Voxel 필터 크기를 동적 조정
  - **매칭 DOP**: 매칭된 포인트의 기하학적 분포를 평가하여 EKF 측정치 공분산을 adaptive하게 설정
- **포즈 그래프 최적화 연동**: 매핑 모드에서 `pose_graph_optimization` 노드와 자동 연동, DOP 기반 루프 클로저 신뢰도 필터링 지원
- **Octomap 연동**: 로컬라이제이션 모드에서 `octomap_server` 노드와 연동하여 실시간 점유 격자 지도 생성
- **다중 LiDAR 지원**: Livox(Avia, MID360, MID70), Velodyne, Ouster(32/64/128), Hesai32
- **성능 통계 출력**: 종료 시 포인트 매칭, KD-Tree, EKF 갱신 등 처리 시간 자동 출력

---

## 알고리즘 개요

### FAST-LIO2 핵심 파이프라인

FAST-LIO2는 IMU 데이터와 LiDAR 포인트 클라우드를 강결합(tightly-coupled)한 반복적 오류 상태 칼만 필터(IESKF)를 기반으로 위치 및 자세를 추정합니다.

1. **IMU 전파(Forward Propagation)**: IMU 측정값을 이용하여 로봇 상태를 예측
2. **LiDAR 모션 보정(Undistortion)**: IMU 적분 결과를 이용하여 스캔 왜곡을 보정
3. **포인트 매칭**: ikd-Tree를 이용한 스캔-투-맵(scan-to-map) 포인트 매칭
4. **IESKF 갱신**: 매칭 잔차를 관측치로 사용하여 상태 갱신 및 공분산 업데이트

### DOP 기반 스캔 매칭 신뢰도 평가

GNSS에서 위성과 수신기 간의 기하학적 배치를 수치화한 DOP 개념을 LiDAR 스캔 매칭에 응용합니다.

**DOP 계산 수식**

각 매칭 포인트 \( i \)에 대해 센서 원점과의 거리:

$$r_i = \sqrt{(x_i - x_s)^2 + (y_i - y_s)^2 + (z_i - z_s)^2}$$

단위 벡터 행렬:

$$A = \begin{bmatrix} \frac{x_1 - x_s}{r_1} & \frac{y_1 - y_s}{r_1} & \frac{z_1 - z_s}{r_1} \\ \vdots & \vdots & \vdots \\ \frac{x_n - x_s}{r_n} & \frac{y_n - y_s}{r_n} & \frac{z_n - z_s}{r_n} \end{bmatrix}$$

공분산 행렬 및 DOP:

$$Q = (A^T A)^{-1}, \quad \text{DOP} = \sqrt{\text{tr}(Q)}$$

**Tukey Loss Function을 통한 스케일링**

$$s(\rho) = \begin{cases} c^2 \left(1 - \left(1 - \left(\frac{\rho}{c}\right)^2\right)^3\right) & \text{if } \rho < c \\ c^2 & \text{otherwise} \end{cases}$$

계산된 \( s(\rho) \)를 LiDAR 측정치 공분산에 곱하여 가중치를 동적으로 조정합니다:

$$R = s(\rho) \cdot \text{diag}(\sigma_L^2)$$

**두 단계 DOP 활용**

| 단계 | 입력 | 활용 방법 |
|------|------|-----------|
| 스캔 DOP | 입력 전체 스캔 포인트 | Voxel 필터 크기 동적 조정 (스캔 품질 불량 시 필터 크기 감소) |
| 매칭 DOP | IESKF 관측에 사용된 매칭 포인트 | EKF 측정치 공분산(`lidar_meas_cov`) 동적 설정 |

---

## 시스템 구성

### 매핑 모드 (Mapping)

```
[LiDAR] ──┐
           ├──► [FAST-LIO2 Node (laserMapping)] ──► /Odometry
[IMU]  ──┘                                     ──► /cloud_registered
                                                ──► /keyframe (키프레임)
                                                       │
                                               [Pose Graph Optimization Node]
                                                       │
                                                ──► /path (최적화된 궤적)
                                                ──► Map/*.pcd (지도 저장)
```

### 로컬라이제이션 모드 (Localization)

```
[LiDAR] ──┐
           ├──► [FAST-LIO2 Localization Node (laserLocalization)] ──► /Odometry
[IMU]  ──┘    ↑                                                  ──► /cloud_registered_body
    [Map/*.pcd]
                                                                          │
                                                                  [Octomap Server Node]
                                                                          │
                                                                   ──► /projected_map
                                                                   ──► /octomap_full
```

---

## 필수 사전 설치

### 1. Ubuntu & ROS2

- **Ubuntu >= 20.04** (권장: Ubuntu 22.04)
- **ROS >= Foxy** (권장: ROS Humble)
  - [ROS Humble 설치](https://docs.ros.org/en/humble/Installation.html)

### 2. PCL & Eigen

```bash
sudo apt install libpcl-dev libeigen3-dev
```

- PCL >= 1.8
- Eigen >= 3.3.4

### 3. livox_ros_driver2

Livox 계열 LiDAR를 사용할 경우 필수 설치:

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble
```

설치 후 `~/.bashrc`에 소스 추가:

```bash
source ~/ws_livox/install/setup.bash
```

### 4. 의존 패키지 (매핑 모드)

매핑 모드는 `pose_graph_optimization` 패키지와 연동됩니다:

```bash
# 동일 워크스페이스에 pose_graph_optimization 패키지 포함 필요
```

### 5. 의존 패키지 (로컬라이제이션 모드)

로컬라이제이션 모드는 `octomap_server` 패키지와 연동됩니다:

```bash
sudo apt install ros-humble-octomap-server
```

---

## 빌드

```bash
cd <ros2_ws>/src
git clone https://github.com/Kimkyuwon/fast_lio2_mapping_and_localization.git --recursive fast_lio
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select fast_lio
source install/setup.bash
```

> **주의**: Livox LiDAR 사용 시 빌드 전 `livox_ros_driver2` 소스가 적용되어 있어야 합니다.

---

## 실행 방법

### 매핑 모드

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mapping_config.yaml
```

LiDAR 종류에 따라 `config_file` 변경:

| LiDAR | config 파일 |
|-------|------------|
| Hesai Pandar 32 | `mapping_config.yaml` |
| Livox Avia | `avia.yaml` |
| Livox MID360 | `mid360.yaml` |
| Livox MID70 | `mid70.yaml` |
| Velodyne | `velodyne.yaml` |
| Ouster 32 | `ouster32.yaml` |
| Ouster 64 | `ouster64.yaml` |
| Ouster 128 | `ouster128.yaml` |

**실제 LiDAR 사용 시 드라이버 별도 실행**:

```bash
# MID360 예시
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**PCD 맵 저장**:
1. `mapping_config.yaml`에서 `pcd_save.pcd_save_en: true` 설정
2. `map_file_path` 경로 지정 (기본: `Map/` 폴더)
3. RQT에서 `/map_save` 서비스 호출하여 저장 트리거

### 로컬라이제이션 모드

```bash
ros2 launch fast_lio localization.launch.py config_file:=localization_config.yaml
```

> `localization_config.yaml`의 `map_file_path`를 사전에 생성한 `.pcd` 파일 경로로 설정해야 합니다.

---

## 파라미터 설정

### 공통 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `common.lid_topic` | LiDAR 포인트 클라우드 토픽 | `/hesai/pandar` |
| `common.imu_topic` | IMU 토픽 | `/alphasense/imu` |
| `common.time_sync_en` | 소프트웨어 시간 동기화 활성화 | `false` |
| `feature_extract_enable` | 특징 추출 활성화 (false: raw point 사용) | `false` |
| `point_filter_num` | 포인트 필터링 간격 | `4` |
| `max_iteration` | IESKF 최대 반복 횟수 | `5` |
| `filter_size_surf` | Voxel 다운샘플링 크기 (m) | `0.4` |

### 전처리 파라미터

| 파라미터 | 설명 | LiDAR 유형 코드 |
|---------|------|----------------|
| `preprocess.lidar_type` | LiDAR 종류 | 1: Livox Avia, 2: Velodyne, 3: Ouster, 4: Hesai |
| `preprocess.scan_line` | 수직 채널 수 | LiDAR 사양에 맞게 설정 |
| `preprocess.blind` | 최소 유효 거리 (m) | `0.5` |

### 매핑 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `mapping.extrinsic_T` | LiDAR→IMU 변환 (위치, m) | `[0,0,0]` |
| `mapping.extrinsic_R` | LiDAR→IMU 변환 (회전 행렬) | Identity |
| `mapping.keyframe_threshold` | 키프레임 생성 거리 임계값 (m) | `1.0` |
| `mapping.dop_flag` | DOP 기반 가중치 조정 활성화 | `true` |

### 로컬라이제이션 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `map_file_path` | 사전 생성 지도 파일 경로 | `Map/map.pcd` |
| `localization.odom_mode` | Odometry 입력 모드 (0: 없음, 1: 2D, 2: 3D) | `2` |
| `localization.dop_flag` | DOP 기반 가중치 조정 활성화 | `true` |

### 포즈 그래프 최적화 파라미터 (매핑 모드)

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `posegraph.r_solid_thres` | 루프 클로저 신뢰도 임계값 | `0.95` |
| `posegraph.loop_dist` | 루프 클로저 탐색 최대 거리 (m) | `80.0` |
| `posegraph.dop_thres` | 루프 클로저 DOP 필터링 임계값 | `1.2` |

---

## 발행 토픽

### 매핑 노드 (`/fastlio_mapping`)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cloud_registered` | `PointCloud2` | 월드 좌표계 변환 포인트 클라우드 |
| `/cloud_registered_body` | `PointCloud2` | 바디 좌표계 포인트 클라우드 |
| `/Odometry` | `Odometry` | 추정된 로봇 Odometry |
| `/path` | `Path` | 추정 궤적 |
| `/effect_cloud` | `PointCloud2` | IESKF에 사용된 유효 포인트 |

### 로컬라이제이션 노드 (`/fastlio_localization`)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cloud_registered_body` | `PointCloud2` | 바디 좌표계 포인트 클라우드 |
| `/Odometry` | `Odometry` | 추정된 로봇 위치 |
| `/path` | `Path` | 추정 궤적 |

---

## 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `<lid_topic>` | `PointCloud2` 또는 `CustomMsg` | LiDAR 포인트 클라우드 |
| `<imu_topic>` | `Imu` | IMU 데이터 |
| `<odom_topic>` | `Odometry` | 외부 Odometry (로컬라이제이션 모드) |

---

## 성능 지표 (DOP 기법 적용 결과)

DOP 기반 스캔 매칭 신뢰도 평가 기법의 성능은 다음 데이터셋에서 검증되었습니다.

### Hilti SLAM Challenge 2022 (ATE, m)

| 방법 | exp-06 RMSE | exp-14 RMSE | exp-16 RMSE | exp-18 RMSE |
|------|------------|------------|------------|------------|
| FAST-LIO2 | 0.039 | 0.058 | 발산 | 1.452 |
| Faster-LIO | 0.070 | 0.090 | 발산 | 4.000 |
| **제안 기법** | **0.045** | 0.077 | **0.636** | **0.156** |

> exp-16, exp-18은 좁은 계단 구간 포함. 기존 방법은 궤적이 발산하는 반면, DOP 기법 적용 시 안정적인 추정 유지.

### VBR SLAM Dataset (ATE, m)

| 방법 | Colosseum RMSE | Diag RMSE |
|------|---------------|----------|
| Faster-LIO | 2.730 | 0.217 |
| Faster-LIO + PGO | 0.386 | 0.179 |
| **제안 기법 (DOP + PGO)** | **0.250** | **0.165** |

---

## 관련 논문

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

## 관련 프로젝트

**SLAM:**
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree): 3D kNN 탐색을 위한 동적 KD-Tree
- [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): FAST-LIO 기반 재로컬라이제이션
- [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init): LiDAR-IMU 외부 파라미터 초기화

**제어 및 계획:**
- [IKFoM](https://github.com/hku-mars/IKFoM): 매니폴드 위 칼만 필터 툴박스

---

## 감사의 말

- [FAST-LIO2 (HKU-MARS)](https://github.com/hku-mars/FAST_LIO): 핵심 LiDAR-Inertial Odometry 알고리즘
- [FAST_LIO_ROS2 (Ericsii)](https://github.com/Ericsii/FAST_LIO_ROS2): ROS2 포팅 기반 코드
- [LOAM](https://github.com/laboshinl/loam_velodyne): LiDAR Odometry 원본 알고리즘 (Ji Zhang, Carnegie Mellon University)
- [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)

---

## 라이선스

BSD License
