# =============================================
# LocAnalytics.msg
# Real-time analytics for FAST_LIO Localization node
# =============================================

# --- Sensor Frequencies (Hz) ---
int64 imu_freq        # IMU data receive frequency
int64 lid_freq        # LiDAR data receive frequency

# --- Scan / Map Statistics ---
int64   scan_size       # Raw input scan point count (feats_undistort)
int64   down_size       # Voxel-downsampled scan point count (feats_down_body)
int64   map_size        # Total points in ikd-Tree (ikdtree.size())
int64   map_valid_size  # Valid (non-deleted) points in ikd-Tree (ikdtree.validnum())
int64   new_idxs        # Points added to map this frame (add_point_size, non-zero only on map update)
int64   buffer_size     # Pending LiDAR buffer size
int64   imu_buffer_size # Pending IMU buffer size
float64 scan_time       # LiDAR scan duration (lidar_end_time - lidar_beg_time, sec)

# --- Adaptive Downsampling Parameters ---
float64 filter_size_surf_ad  # Adaptive voxel leaf size for surface downsampling (m)
int64   point_filter_num_ad  # Adaptive point stride for pre-scan thinning

# --- Feature Matching ---
int64   num_feats    # Effective matched surface feature count (effct_feat_num)
int64   num_reject   # Rejected point count (down_size - num_feats)
float64 match_ratio  # Feature matching success ratio (num_feats / down_size, 0~1)

# --- Residual Statistics (from matched features) ---
float64 res_mean  # Mean point-to-plane residual
float64 res_std   # Residual standard deviation

# --- IESEKF Optimization ---
float64 pos_cov        # EKF position covariance trace (P[0,0]+P[1,1]+P[2,2])
float64 rot_cov        # EKF rotation covariance trace (P[3,3]+P[4,4]+P[5,5])
float64 lidar_meas_cov # Tukey-loss adaptive LiDAR measurement covariance (lidar_meas_cov)

# --- Geometry Quality (DOP: lower is better) ---
float64 scan_dop      # PDOP of full scan point distribution (feats_undistort)
float64 down_dop      # PDOP of downsampled scan point distribution (feats_down_body)
float64 matching_dop  # PDOP of matched feature point distribution
float64 dop_ratio     # Localization quality ratio (matching_dop / down_dop, ~1.0 is good)

# --- IMU State Estimates ---
float64 vel_norm      # Estimated velocity magnitude (state_point.vel.norm(), m/s)
float64 acc_bias_norm # Accelerometer bias magnitude (state_point.ba.norm())
float64 gyr_bias_norm # Gyroscope bias magnitude (state_point.bg.norm())

# --- Time Sync Offsets (sec) ---
float64 lid_offset # LiDAR hardware time offset (time_diff_lidar_to_imu)
float64 imu_offset # IMU-LiDAR real-time sync error (timediff_lidar_wrt_imu)

# --- Per-frame Processing Time (sec) ---
float64 imu_time   # IMU undistortion time (p_imu->Process)
float64 state_time # EKF state update time (update_iterated_dyn_share_modified)
float64 map_time   # Map incremental update time (map_incremental, 0 when not updated)
float64 total_time # Total pipeline time per frame

# --- Cumulative Timing Statistics (sec) ---
float64 imu_mean   # Mean IMU undistortion time
float64 state_mean # Mean EKF state update time
float64 map_mean   # Mean map update time
float64 total_mean # Mean total pipeline time

float64 imu_max    # Max IMU undistortion time
float64 state_max  # Max EKF state update time
float64 map_max    # Max map update time
float64 total_max  # Max total pipeline time

# --- Keyframe Info ---
int64   kf_count      # Cumulative keyframe count (kf_idx_)
float64 kf_dist_last  # Distance traveled since last keyframe (m)

# --- System Resources ---
int64 run_time   # Node uptime since first scan (lidar_end_time - first_lidar_time, sec)
float64 traj_dist  # Total trajectory distance traveled (m)
int64 ram_usage  # Process RSS memory usage (MB)
int64 cpu_usage  # Process CPU usage (%, normalized by core count)

# --- Localization Status ---
bool    is_initialized  # Localization initialized flag (initial_flag)
bool    map_updated     # Map was updated this frame (dop_ratio > 1.4)

# --- Wheel Odometry Input ---
float64 linear_velo   # Wheel odometry linear velocity (m/s)
float64 angular_velo  # Wheel odometry angular velocity (rad/s)
