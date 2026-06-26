// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "fast_lio/msg/lio_analytics.hpp"
#include <sys/times.h>
#include <sys/resource.h>
#include <cfloat>


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
#define DOP_VOXEL_SIZE      (2.0)
#define MIN_VALID_DOP       (100.0)
#define TUKEY_LOSS_C        (3.0)
#define MAX_PATH_LENGTH     (5000)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double total_match_time = 0.0;
int match_count = 0;
double total_kdtree_incremental_time = 0.0;
int kdtree_incremental_count = 0;
double total_solve_time = 0.0;
int solve_time_count = 0;
double total_solve_H_time = 0.0;
int solve_H_time_count = 0;
double total_removal_time = 0.0;
int removal_time_count = 0;
double total_process_time = 0.0;
int process_count = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

/*** Analytics Variables ***/
// Sensor frequency tracking (count per ~1s window)
double  analytics_freq_t0_     = -1.0;
int     analytics_imu_cnt_w_   = 0, analytics_lid_cnt_w_ = 0;
int64_t analytics_imu_freq_    = 0, analytics_lid_freq_  = 0;

// Per-frame timing (seconds)
double analytics_imu_time_    = 0.0, analytics_state_time_ = 0.0;
double analytics_map_time_    = 0.0, analytics_total_time_ = 0.0;
double analytics_search_time_ = 0.0; // accumulated per h_share_model call
int    analytics_kf_iter_cnt_ = 0;   // IESEKF iteration count per frame

// Cumulative timing stats
int    analytics_frame_cnt_   = 0;
double analytics_mean_imu_    = 0.0, analytics_mean_state_ = 0.0;
double analytics_mean_map_    = 0.0, analytics_mean_total_ = 0.0;
double analytics_max_imu_     = 0.0, analytics_max_state_  = 0.0;
double analytics_max_map_     = 0.0, analytics_max_total_  = 0.0;

// Trajectory & keyframe
double analytics_start_time_  = -1.0;
double analytics_traj_dist_   = 0.0;
V3D    analytics_prev_pos_    = V3D::Zero();
bool   analytics_pos_init_    = false;

// Residual statistics (beyond mean, computed from res_last[])
double analytics_res_std_ = 0.0, analytics_res_min_ = 0.0, analytics_res_max_ = 0.0;

// CPU usage tracking
struct tms analytics_last_tms_ = {};
double     analytics_last_wall_ = 0.0;
int        analytics_num_proc_  = 1;
bool       analytics_cpu_init_  = false;
/****************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0, filter_size_surf_ad = 0, filter_size_map_min = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, point_filter_num = 0, point_filter_num_ad = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, scan_body_pub_en = false;
bool   is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
vector<double>       extrin_g2o_T(3, 0.0);
vector<double>       extrin_g2o_R(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
V3D Odom_T_wrt_LIDAR(Zero3d);
M3D Odom_R_wrt_LIDAR(Eye3d);



/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;
vect3 prev_kf_pos;

//  key frame   //
double kf_thres_;
bool kf_flag = false;
int kf_idx_ = 0;

//  DOP //
bool dop_flag = false;
double scan_dop, matching_dop, lidar_meas_cov;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());


void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig " << sig << std::endl;
    
    // 평균 시간 통계 출력
    std::cout << "===========================================" << std::endl;
    std::cout << "FAST-LIO Performance Statistics:" << std::endl;
    std::cout << "===========================================" << std::endl;
    
    if (match_count > 0) {
        double average_match_time = total_match_time / match_count;
        std::cout << "Point Cloud Matching:" << std::endl;
        std::cout << "  Total operations: " << match_count << std::endl;
        std::cout << "  Average time: " << average_match_time * 1000.0 << " ms" << std::endl;
    }
    
    if (kdtree_incremental_count > 0) {
        double average_kdtree_time = total_kdtree_incremental_time / kdtree_incremental_count;
        std::cout << "KDTree Incremental:" << std::endl;
        std::cout << "  Total operations: " << kdtree_incremental_count << std::endl;
        std::cout << "  Average time: " << average_kdtree_time * 1000.0 << " ms" << std::endl;
    }
    
    if (solve_time_count > 0) {
        double average_solve_time = total_solve_time / solve_time_count;
        std::cout << "Jacobian Matrix Computation:" << std::endl;
        std::cout << "  Total operations: " << solve_time_count << std::endl;
        std::cout << "  Average time: " << average_solve_time * 1000.0 << " ms" << std::endl;
    }
    
    if (solve_H_time_count > 0) {
        double average_solve_H_time = total_solve_H_time / solve_H_time_count;
        std::cout << "EKF Update (solve_H):" << std::endl;
        std::cout << "  Total operations: " << solve_H_time_count << std::endl;
        std::cout << "  Average time: " << average_solve_H_time * 1000.0 << " ms" << std::endl;
    }
    
    if (process_count > 0) {
        double average_porocess_time = total_process_time / process_count;
        std::cout << "All Process:" << std::endl;
        std::cout << "  Total operations: " << process_count << std::endl;
        std::cout << "  Average time: " << average_porocess_time * 1000.0 << " ms" << std::endl;
    }
    std::cout << "===========================================" << std::endl;
    
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// New function: Transform point from LiDAR body to body frame world coordinates
void RGBpointLidarToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    // Transform from LiDAR to IMU frame first
    V3D p_imu = state_point.offset_R_L_I*p_body + state_point.offset_T_L_I;
    // Transform from IMU to world frame
    V3D p_global = state_point.rot * p_imu + state_point.pos;

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// New function: Transform point from LiDAR frame to body frame local coordinates
void RGBpointLidarToBodyFrame(PointType const * const pi, PointType * const po)
{
    V3D p_lidar(pi->x, pi->y, pi->z);
    // Transform from LiDAR to IMU frame first
    V3D p_body = state_point.offset_R_L_I*p_lidar + state_point.offset_T_L_I;

    po->x = p_body(0);
    po->y = p_body(1);
    po->z = p_body(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    analytics_lid_cnt_w_++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    analytics_lid_cnt_w_++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    analytics_imu_cnt_w_++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
    
    // kdtree_incremental_time 누적
    total_kdtree_incremental_time += kdtree_incremental_time;
    kdtree_incremental_count++;
}

double computeDOP(const PointCloudXYZI::Ptr& cloud, Eigen::Vector3d pos)
{
    PointCloudXYZI::Ptr dop_cloud(new PointCloudXYZI());
    pcl::VoxelGrid<PointType> downSizeFilterDOP;
    downSizeFilterDOP.setLeafSize(DOP_VOXEL_SIZE, DOP_VOXEL_SIZE, DOP_VOXEL_SIZE);
    downSizeFilterDOP.setInputCloud(cloud);
    downSizeFilterDOP.filter(*dop_cloud);  

    std::vector<Eigen::Vector3d> range_info;
    for (size_t k = 0; k < dop_cloud->points.size(); k++)
    {
        double r = std::sqrt(std::pow((dop_cloud->points[k].x-pos(0)), 2) + std::pow((dop_cloud->points[k].y-pos(1)), 2) + std::pow((dop_cloud->points[k].z-pos(2)), 2));
        if (r < p_pre->blind || std::isnan(r))    continue;
        Eigen::Vector3d r_info;
        r_info(0) = dop_cloud->points[k].x / r;
        r_info(1) = dop_cloud->points[k].y / r;
        r_info(2) = dop_cloud->points[k].z / r;
        range_info.push_back(r_info);    
    }
    Eigen::MatrixXd AA(range_info.size(), 3);
    for (size_t p = 0; p < range_info.size(); p++)
    {
        AA(p, 0) = range_info[p](0);
        AA(p, 1) = range_info[p](1);
        AA(p, 2) = range_info[p](2);
    }
    Eigen::Matrix3d A_sq;
    Eigen::Matrix3d Q;
    A_sq = AA.transpose() * AA;
    Q = A_sq.inverse();

    double pdop = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
    if (pdop == 0 || pdop > MIN_VALID_DOP || std::isnan(pdop))
    {
        pdop = MIN_VALID_DOP;
    }
    return pdop;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointLidarToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointLidarToBodyFrame(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "base_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointLidarToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void PublishKeyFrame(rclcpp::Publisher<fast_lio::msg::Frame>::SharedPtr pubKeyFrame)
{
    Frame kfMsg;
    kfMsg.header.stamp = get_ros_time(lidar_end_time);
    kfMsg.pose = odomAftMapped;

    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laser_cloud_body_local(new PointCloudXYZI(size, 1));

    // Transform pointcloud to body frame local coordinates for pose graph optimization
    for (int i = 0; i < size; i++) {
        RGBpointLidarToBodyFrame(&feats_undistort->points[i], &laser_cloud_body_local->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_body_local, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "base_link";  // Local body frame for pose graph optimization

    kfMsg.pointcloud = laserCloudmsg;
    kfMsg.frame_idx = kf_idx_;
    pubKeyFrame->publish(kfMsg);
    kf_idx_++;
}


void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(feats_undistort);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointLidarToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "odom";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    // Transform from IMU frame to base_link frame using extrinsic_g2o parameters
    V3D pos_imu = state_point.pos;
    M3D rot_imu = state_point.rot.toRotationMatrix();

    // Convert rotation matrix to quaternion
    Eigen::Quaterniond quat_imu(rot_imu);

    out.pose.position.x = pos_imu(0);
    out.pose.position.y = pos_imu(1);
    out.pose.position.z = pos_imu(2);
    out.pose.orientation.x = quat_imu.x();
    out.pose.orientation.y = quat_imu.y();
    out.pose.orientation.z = quat_imu.z();
    out.pose.orientation.w = quat_imu.w();

}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "base_link";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    // if (jjj % 10 == 0) 
    // {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    // }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    analytics_kf_iter_cnt_++;
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/

    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    PointCloudXYZI::Ptr dop_cloud(new PointCloudXYZI());
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
            
            if (dop_flag)
            {
                dop_cloud->push_back(feats_down_body->points[i]); 
            }
        }
    }
    
    if (dop_flag)
    {
        matching_dop = computeDOP(dop_cloud, Eigen::Vector3d(0,0,0));
        double dop_scale = 1;
        double c = 50.0;
        double r = matching_dop;

        //tukey loss function
        double scale = std::pow(c,2)*(1-std::pow((1-std::pow((r/c),2)),3))*dop_scale;
        if (r >= c)  scale = std::pow(c,2)*dop_scale;   

        lidar_meas_cov = scale * LASER_POINT_COV;
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    double current_match_time = omp_get_wtime() - match_start;
    
    // 매칭 시간 누적 및 카운트 증가
    total_match_time += current_match_time;
    match_count++;
    analytics_search_time_ += current_match_time;
    
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    /*** LIDAR Part ***/
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measurement Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 15>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 15>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measurement: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

    double current_solve_time = omp_get_wtime() - solve_start_;
    
    // solve_time 누적
    total_solve_time += current_solve_time;
    solve_time_count++;
}

// ---------- Analytics helper functions ----------

double ComputeRamUsageMB()
{
    long rss = 0;
    FILE *fp = fopen("/proc/self/status", "r");
    if (!fp) return 0.0;
    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "VmRSS:", 6) == 0) {
            sscanf(line + 6, "%ld", &rss);
            break;
        }
    }
    fclose(fp);
    return rss / 1024.0;  // kB → MB
}

double ComputeCpuPercent()
{
    struct tms cur_tms;
    double wall = omp_get_wtime();
    times(&cur_tms);
    if (!analytics_cpu_init_) {
        analytics_last_tms_  = cur_tms;
        analytics_last_wall_ = wall;
        analytics_cpu_init_  = true;
        return 0.0;
    }
    double elapsed = wall - analytics_last_wall_;
    if (elapsed <= 0.0) return 0.0;
    double cpu_ticks =
        (double)((cur_tms.tms_utime - analytics_last_tms_.tms_utime) +
                 (cur_tms.tms_stime - analytics_last_tms_.tms_stime));
    double percent = (cpu_ticks / sysconf(_SC_CLK_TCK)) / elapsed
                     / analytics_num_proc_ * 100.0;
    analytics_last_tms_  = cur_tms;
    analytics_last_wall_ = wall;
    return percent;
}

// ------------------------------------------------

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        // QoS for visualization topics (latest data only)
        auto qos_viz = rclcpp::QoS(rclcpp::KeepLast(1));
        qos_viz.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_viz.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // QoS for SLAM data processing (buffered, no data loss)
        auto qos_slam = rclcpp::QoS(rclcpp::KeepLast(10));  
        qos_slam.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // 메시지 손실 없음
        qos_slam.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        path_en = this->declare_parameter("publish.path_en", true);
        effect_pub_en = this->declare_parameter("publish.effect_map_en", false);
        map_pub_en = this->declare_parameter("publish.map_en", false);
        scan_pub_en = this->declare_parameter("publish.scan_publish_en", true);
        scan_body_pub_en = this->declare_parameter("publish.scan_bodyframe_pub_en", true);
        NUM_MAX_ITERATIONS = this->declare_parameter("max_iteration", 4);
        map_file_path = this->declare_parameter("map_file_path", "");
        lid_topic = this->declare_parameter("common.lid_topic", "/livox/lidar");
        imu_topic = this->declare_parameter("common.imu_topic", "/livox/imu");
        time_sync_en = this->declare_parameter("common.time_sync_en", false);
        time_diff_lidar_to_imu = this->declare_parameter("common.time_offset_lidar_to_imu", 0.0);
        filter_size_surf_min = this->declare_parameter("filter_size_surf", 0.5);
        filter_size_map_min = this->declare_parameter("filter_size_map", 0.5);
        cube_len = this->declare_parameter("cube_side_length", 200.);
        DET_RANGE = this->declare_parameter("mapping.det_range", 300.);
        gyr_cov = this->declare_parameter("mapping.gyr_cov", 0.1);
        acc_cov = this->declare_parameter("mapping.acc_cov", 0.1);
        b_gyr_cov = this->declare_parameter("mapping.b_gyr_cov", 0.0001);
        b_acc_cov = this->declare_parameter("mapping.b_acc_cov", 0.0001);
        kf_thres_ = this->declare_parameter("mapping.keyframe_threshold", 0.5);
        dop_flag = this->declare_parameter("mapping.dop_flag", false);
        p_pre->blind = this->declare_parameter("preprocess.blind", 0.01);
        p_pre->lidar_type = this->declare_parameter("preprocess.lidar_type", 0);
        p_pre->N_SCANS = this->declare_parameter("preprocess.scan_line", 16);
        p_pre->time_unit = this->declare_parameter("preprocess.timestamp_unit", 3);
        p_pre->SCAN_RATE = this->declare_parameter("preprocess.scan_rate", 10);
        point_filter_num = this->declare_parameter("point_filter_num", 2);
        p_pre->feature_enabled = this->declare_parameter("feature_extract_enable", false);
        extrinsic_est_en = this->declare_parameter("mapping.extrinsic_est_en", true);
        pcd_save_en = this->declare_parameter("pcd_save.pcd_save_en", false);
        pcd_save_interval = this->declare_parameter("pcd_save.interval", -1);
        extrinT = this->declare_parameter("mapping.extrinsic_T", vector<double>());
        extrinR = this->declare_parameter("mapping.extrinsic_R", vector<double>());
        extrin_g2o_T = this->declare_parameter("mapping.extrinsic_g2o_T", vector<double>());
        extrin_g2o_R = this->declare_parameter("mapping.extrinsic_g2o_R", vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="odom";

        
        lidar_meas_cov = LASER_POINT_COV;

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        filter_size_surf_ad = filter_size_surf_min;
        point_filter_num_ad = point_filter_num;

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, qos_slam, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, qos_slam, standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos_slam, imu_cbk);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos_slam);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", qos_viz);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", qos_slam);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", qos_viz);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", qos_slam);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", qos_slam);
        pubKeyFrame = this->create_publisher<fast_lio::msg::Frame> ("/key_frame", qos_slam);
        pub_analytics_ = this->create_publisher<fast_lio::msg::LioAnalytics>(
            "/lio_analytics", rclcpp::QoS(rclcpp::KeepLast(1)));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        analytics_num_proc_ = (int)sysconf(_SC_NPROCESSORS_ONLN);
        if (analytics_num_proc_ <= 0) analytics_num_proc_ = 1;

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(10.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
    }

private:
    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }
            
            double process_start = omp_get_wtime();


            double t_imu_start = omp_get_wtime();
            p_imu->Process(Measures, kf, feats_undistort);
            analytics_imu_time_ = omp_get_wtime() - t_imu_start;
            state_point = kf.get_x();

            /*** Transform feats_undistort from LiDAR frame to body frame ***/
            Eigen::Affine3f body_TF = Eigen::Affine3f::Identity();
            body_TF.linear() = Odom_R_wrt_LIDAR.cast<float>();
            body_TF.translation() = Odom_T_wrt_LIDAR.cast<float>();
            pcl::transformPointCloud(*feats_undistort, *feats_undistort, body_TF);

            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;


            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            if (dop_flag)
            {
                scan_dop = computeDOP(feats_undistort, Eigen::Vector3d(0,0,0));
                double dop_scale = 1;
                double c = TUKEY_LOSS_C;
                double r = scan_dop;

                //tukey loss function
                double scale = std::pow(c,2)*(1-std::pow((1-std::pow((r/c),2)),3))*dop_scale;
                if (r >= c)  scale = std::pow(c,2)*dop_scale; 
                
                double voxel_scale = 1.0/scale;
                filter_size_surf_ad = voxel_scale * filter_size_surf_min;
                if (filter_size_surf_ad < 0.05)  filter_size_surf_ad = 0.05;
                else if (filter_size_surf_ad > 1.0) filter_size_surf_ad = 1.0;
                
                point_filter_num_ad = static_cast<int>(voxel_scale * point_filter_num);
                if (point_filter_num_ad <= 1)  point_filter_num_ad = 1;
                else if (point_filter_num_ad >= p_pre->N_SCANS / 2)   point_filter_num_ad = static_cast<int>(p_pre->N_SCANS / 2);
            }

            /*** downsample the feature points in a scan ***/
            PointCloudXYZI::Ptr temp_pointCloud(new PointCloudXYZI());
            for (int i = 0; i < feats_undistort->points.size(); i++)
            {
                if (i % point_filter_num_ad == 0) 
                {
                    temp_pointCloud->points.push_back(feats_undistort->points[i]);
                }
            }

            
            downSizeFilterSurf.setLeafSize(filter_size_surf_ad, filter_size_surf_ad, filter_size_surf_ad);
            downSizeFilterSurf.setInputCloud(temp_pointCloud);
            downSizeFilterSurf.filter(*feats_down_body);

            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            
            /*** iterated state estimation ***/
            analytics_kf_iter_cnt_ = 0;
            analytics_search_time_ = 0.0;
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(lidar_meas_cov, solve_H_time);
            analytics_state_time_ = omp_get_wtime() - t_update_start;

            // solve_H_time 누적
            total_solve_H_time += solve_H_time;
            solve_H_time_count++;
            
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            double t_map_start = omp_get_wtime();
            map_incremental();
            analytics_map_time_ = omp_get_wtime() - t_map_start;
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /******* Publish key frame *******/
            double kf_dist = (pos_lid - prev_kf_pos).norm();
            if (kf_dist > kf_thres_ || !kf_flag)
            {
                PublishKeyFrame(pubKeyFrame);
                prev_kf_pos = pos_lid;
                kf_flag = true;
            }   

            double current_process_time = omp_get_wtime() - process_start;
            total_process_time += current_process_time;
            process_count++;

            // ------ Analytics: collect per-frame data ------
            if (analytics_start_time_ < 0.0)
                analytics_start_time_ = omp_get_wtime();

            analytics_total_time_ = analytics_imu_time_ + analytics_state_time_ + analytics_map_time_;

            // Cumulative timing
            analytics_frame_cnt_++;
            analytics_mean_imu_   += (analytics_imu_time_   - analytics_mean_imu_)   / analytics_frame_cnt_;
            analytics_mean_state_ += (analytics_state_time_ - analytics_mean_state_) / analytics_frame_cnt_;
            analytics_mean_map_   += (analytics_map_time_   - analytics_mean_map_)   / analytics_frame_cnt_;
            analytics_mean_total_ += (analytics_total_time_ - analytics_mean_total_) / analytics_frame_cnt_;
            if (analytics_imu_time_   > analytics_max_imu_)   analytics_max_imu_   = analytics_imu_time_;
            if (analytics_state_time_ > analytics_max_state_) analytics_max_state_ = analytics_state_time_;
            if (analytics_map_time_   > analytics_max_map_)   analytics_max_map_   = analytics_map_time_;
            if (analytics_total_time_ > analytics_max_total_) analytics_max_total_ = analytics_total_time_;

            // Trajectory distance
            if (!analytics_pos_init_) {
                analytics_prev_pos_ = state_point.pos;
                analytics_pos_init_ = true;
            } else {
                analytics_traj_dist_ += (state_point.pos - analytics_prev_pos_).norm();
                analytics_prev_pos_  = state_point.pos;
            }

            // Residual statistics from res_last[]
            if (effct_feat_num > 1) {
                float r_min = FLT_MAX, r_max = -FLT_MAX;
                double r_sum = 0.0;
                for (int i = 0; i < effct_feat_num; i++) {
                    r_sum += res_last[i];
                    if (res_last[i] < r_min) r_min = res_last[i];
                    if (res_last[i] > r_max) r_max = res_last[i];
                }
                double r_mean = r_sum / effct_feat_num;
                double var = 0.0;
                for (int i = 0; i < effct_feat_num; i++) {
                    double d = res_last[i] - r_mean;
                    var += d * d;
                }
                analytics_res_std_ = std::sqrt(var / (effct_feat_num - 1));
                analytics_res_min_ = r_min;
                analytics_res_max_ = r_max;
            }

            PublishAnalytics();
            // -----------------------------------------------
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void PublishAnalytics()
    {
        double now = omp_get_wtime();

        // Sensor frequency: update once per ~1 second window
        if (analytics_freq_t0_ < 0.0) analytics_freq_t0_ = now;
        double elapsed_freq = now - analytics_freq_t0_;
        if (elapsed_freq >= 1.0) {
            analytics_imu_freq_ = (int64_t)std::round(analytics_imu_cnt_w_ / elapsed_freq);
            analytics_lid_freq_ = (int64_t)std::round(analytics_lid_cnt_w_ / elapsed_freq);
            analytics_imu_cnt_w_ = 0;
            analytics_lid_cnt_w_ = 0;
            analytics_freq_t0_   = now;
        }

        fast_lio::msg::LioAnalytics msg;

        // --- Sensor frequencies ---
        msg.imu_freq = analytics_imu_freq_;
        msg.lid_freq = analytics_lid_freq_;

        // --- Scan / map statistics ---
        msg.scan_size      = (int64_t)feats_undistort->points.size();
        msg.down_size      = (int64_t)feats_down_size;
        msg.map_size       = (int64_t)kdtree_size_st;
        msg.map_valid_size = (int64_t)ikdtree.validnum();
        msg.new_idxs       = (int64_t)add_point_size;
        msg.map_delete_size = (int64_t)kdtree_delete_counter;
        {
            std::unique_lock<std::mutex> lk(mtx_buffer, std::try_to_lock);
            msg.buffer_size     = lk.owns_lock() ? (int64_t)lidar_buffer.size() : -1;
            msg.imu_buffer_size = lk.owns_lock() ? (int64_t)imu_buffer.size()   : -1;
        }
        msg.scan_time = lidar_mean_scantime;

        // --- Feature matching ---
        msg.num_feats   = (int64_t)effct_feat_num;
        msg.num_reject  = (int64_t)(feats_down_size - effct_feat_num);
        msg.match_ratio = (feats_down_size > 0)
                          ? (double)effct_feat_num / feats_down_size : 0.0;

        // --- Residual statistics ---
        msg.res_mean = res_mean_last;
        msg.res_std  = analytics_res_std_;

        // --- IESEKF ---
        msg.kf_iterations = (int64_t)analytics_kf_iter_cnt_;
        {
            auto P = kf.get_P();
            // In esekfom state order: rot[0-2], pos[3-5]
            msg.pos_cov = P(3, 3) + P(4, 4) + P(5, 5);
            msg.rot_cov = P(0, 0) + P(1, 1) + P(2, 2);
        }

        // --- Geometry quality (DOP) ---
        msg.scan_dop     = dop_flag ? scan_dop     : 0.0;
        msg.matching_dop = dop_flag ? matching_dop : 0.0;

        // --- IMU state estimates ---
        msg.vel_norm      = state_point.vel.norm();
        msg.acc_bias_norm = state_point.ba.norm();
        msg.gyr_bias_norm = state_point.bg.norm();

        // --- Time sync offsets ---
        msg.lid_offset = time_diff_lidar_to_imu;
        msg.imu_offset = timediff_lidar_wrt_imu;

        // --- Per-frame processing time ---
        msg.imu_time    = analytics_imu_time_;
        msg.state_time  = analytics_state_time_;
        msg.map_time    = analytics_map_time_;
        msg.total_time  = analytics_total_time_;
        msg.search_time = analytics_search_time_;
        msg.delete_time = kdtree_delete_time;

        // --- Cumulative timing statistics ---
        msg.imu_mean   = analytics_mean_imu_;
        msg.state_mean = analytics_mean_state_;
        msg.map_mean   = analytics_mean_map_;
        msg.total_mean = analytics_mean_total_;
        msg.imu_max    = analytics_max_imu_;
        msg.state_max  = analytics_max_state_;
        msg.map_max    = analytics_max_map_;
        msg.total_max  = analytics_max_total_;

        // --- Keyframe ---
        msg.kf_count     = (int64_t)kf_idx_;
        msg.kf_dist_last = (pos_lid - prev_kf_pos).norm();

        // --- System resources ---
        msg.run_time  = (int64_t)std::round(now - analytics_start_time_);
        msg.traj_dist = (int64_t)std::round(analytics_traj_dist_);
        msg.ram_usage = (int64_t)std::round(ComputeRamUsageMB());
        msg.cpu_usage = (int64_t)std::round(ComputeCpuPercent());

        pub_analytics_->publish(msg);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<fast_lio::msg::Frame>::SharedPtr pubKeyFrame;
    rclcpp::Publisher<fast_lio::msg::LioAnalytics>::SharedPtr pub_analytics_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    return 0;
}
