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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
#define DOP_VOXEL_SIZE      (2.0)
#define MIN_VALID_DOP       (100.0)
#define TUKEY_LOSS_C        (3.0)
#define MAX_PATH_LENGTH     (5000)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0;
double match_time = 0;
double total_match_time = 0.0;
int match_count = 0;
double total_kdtree_incremental_time = 0.0;
int kdtree_incremental_count = 0;
double total_solve_time = 0.0;
int solve_time_count = 0;
double total_solve_H_time = 0.0;
int solve_H_time_count = 0;
double total_process_time = 0.0;
int process_count = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic, odom_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0, filter_size_surf_ad = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
double linear_velo, angular_velo, diff_vel;
double dt, curr_odom_time, prev_odom_time;
int    effct_feat_num = 0, point_filter_num = 0, point_filter_num_ad = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0;
int    odom_mode = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   is_first_lidar = true;
bool   odom_flag = false, initial_flag = false;

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
PointCloudXYZI::Ptr laserCloudMap(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
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
double scan_dop, down_dop, matching_dop, dop_ratio, lidar_meas_cov;

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
    std::cout << "Localization Performance Statistics:" << std::endl;
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
    // Transform from IMU world frame to body world frame
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

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
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

void init_pose_cbk(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr &msg) 
{
    if (linear_velo > 0.01 || std::fabs(angular_velo) > deg2rad(0.5))
    {             
        cout<<"Cannot init Position."<<endl;
        return;
    }
    p_imu->get_init(true);

    geometry_msgs::msg::PoseWithCovarianceStamped poseStamped = *msg;
    
    state_point = kf.get_x();
    state_point.pos(0) = poseStamped.pose.pose.position.x;
    state_point.pos(1) = poseStamped.pose.pose.position.y;
    state_point.pos(2) = poseStamped.pose.pose.position.z;
    
    state_point.rot.coeffs()[0] = poseStamped.pose.pose.orientation.x;
    state_point.rot.coeffs()[1] = poseStamped.pose.pose.orientation.y;
    state_point.rot.coeffs()[2] = poseStamped.pose.pose.orientation.z;
    state_point.rot.coeffs()[3] = poseStamped.pose.pose.orientation.w;
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    state_point.vel.setZero();
    kf.change_x(state_point);
    path.poses.clear();

    initial_flag = false;
    cout<<"Initial pose is Received."<<endl;
}

void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &odom) 
{
    state_point = kf.get_x();
    tf2::Quaternion q;
    q.setW(state_point.rot.w());
    q.setX(state_point.rot.x());
    q.setY(state_point.rot.y());
    q.setZ(state_point.rot.z());

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // 이미 -π~π 범위로 반환

    double yaw_offset = atan2(Odom_R_wrt_LIDAR(1, 0), Odom_R_wrt_LIDAR(0, 0));
    yaw = fmod(yaw + yaw_offset + M_PI, 2 * M_PI) - M_PI;

    if (!odom_flag) 
    {
        odom_flag = true;
        prev_odom_time = get_time_sec(odom->header.stamp);
        return;
    }
    double vel_x, vel_y, vel_x_dt, vel_y_dt;

    curr_odom_time = get_time_sec(odom->header.stamp);
    dt = curr_odom_time - prev_odom_time;
    angular_velo = odom->twist.twist.angular.z;

    /////////////////////////////////////////////
    // odom_mode : 1 -> Linear Velocity only  //
    // odom_mode : 2 -> X/Y Velocity         //
    //////////////////////////////////////////

    if (odom_mode == 1)    
    {
        linear_velo = odom->twist.twist.linear.x;
        yaw += angular_velo * dt;
        vel_x = linear_velo*cos(yaw); vel_y = linear_velo*sin(yaw);
        vel_x_dt = linear_velo*cos(yaw) * dt; vel_y_dt = linear_velo*sin(yaw) * dt;
    }
    else if (odom_mode == 2)
    {
        double body_vel_x = odom->twist.twist.linear.x;
        double body_vel_y = odom->twist.twist.linear.y;
        vel_x = body_vel_x * cos(yaw) - body_vel_y * sin(yaw);
        vel_y = body_vel_x * sin(yaw) + body_vel_y * cos(yaw);
        vel_x_dt = vel_x * dt; vel_y_dt = vel_y * dt;
        linear_velo = sqrt(pow(body_vel_x,2) + pow(body_vel_y,2));
    }
    else    
    {
        return;
    }

    if (linear_velo < 0.01 && std::fabs(angular_velo) < deg2rad(0.5))
    {             
        return;
    }
    // diff_vel = sqrt(pow(state_point.vel(0)-vel_x,2)+pow(state_point.vel(1)-vel_y,2));

    // state_point.pos(0) += vel_x_dt; state_point.pos(1) += vel_y_dt;
    // state_point.vel(0) = vel_x; state_point.vel(1) = vel_y;

    // double dyaw = angular_velo * dt;
    // Eigen::Quaterniond q_current(
    //     state_point.rot.coeffs()[3],
    //     state_point.rot.coeffs()[0],
    //     state_point.rot.coeffs()[1],
    //     state_point.rot.coeffs()[2]);
    // Eigen::Quaterniond q_delta(Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()));
    // Eigen::Quaterniond q_new = (q_current * q_delta).normalized();
    // state_point.rot.coeffs()[0] = q_new.x();
    // state_point.rot.coeffs()[1] = q_new.y();
    // state_point.rot.coeffs()[2] = q_new.z();
    // state_point.rot.coeffs()[3] = q_new.w();
    // kf.change_x(state_point);
    // if (diff_vel > 1.0)
    // {
    //     // state_point.vel(0) = vel_x; state_point.vel(1) = vel_y; state_point.vel(2) = 0;
    //     // kf.change_x(state_point);
    // }

    prev_odom_time = curr_odom_time;
}

void tf_static_cbk(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
{
    if (msg->transforms.empty()) return;

    const auto &transform = msg->transforms[0];

    Odom_T_wrt_LIDAR = V3D(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);

    const auto &q = transform.transform.rotation;
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 tf_mat(tf_q);
    Odom_R_wrt_LIDAR <<
        tf_mat[0][0], tf_mat[0][1], tf_mat[0][2],
        tf_mat[1][0], tf_mat[1][1], tf_mat[1][2],
        tf_mat[2][0], tf_mat[2][1], tf_mat[2][2];

    RCLCPP_INFO(rclcpp::get_logger("laserLocalization"),
        "[tf_static] parent: %s, child: %s | T=(%.3f, %.3f, %.3f)",
        transform.header.frame_id.c_str(),
        transform.child_frame_id.c_str(),
        Odom_T_wrt_LIDAR.x(), Odom_T_wrt_LIDAR.y(), Odom_T_wrt_LIDAR.z());
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
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

int process_increments = 0;
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

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
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
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudMap, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudmsg);
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
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
    if (path.poses.size() > MAX_PATH_LENGTH)
    {
        path.poses.erase(path.poses.begin());
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 

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
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[0] > 2.0*filter_size_surf_ad ? false : true;
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
        dop_ratio = matching_dop/down_dop;
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        return;
    }

    if (!initial_flag)
    {
        if (dop_ratio <= 1.2 )
        {
            cout<<"Position is Initialized."<<endl;
            initial_flag = true;
        }
        else
        {
            ekfom_data.valid = false;       
            s.vel.setZero(); 
            return;
        }
    }

    double current_match_time = omp_get_wtime() - match_start;
    match_time  += current_match_time;
    
    // 매칭 시간 누적 및 카운트 증가
    total_match_time += current_match_time;
    match_count++;
    
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 15);
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
            ekfom_data.h_x.block<1, 15>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C), 0.0, 0.0, 0.0;
        }
        else
        {
            ekfom_data.h_x.block<1, 15>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measurement: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

    double current_solve_time = omp_get_wtime() - solve_start_;
    
    // solve_time 누적
    total_solve_time += current_solve_time;
    solve_time_count++;
}

class LaserLocalizationNode : public rclcpp::Node
{
public:
    LaserLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_localization", options)
    {
        // QoS for visualization topics (latest data only)
        auto qos_viz = rclcpp::QoS(rclcpp::KeepLast(1));
        qos_viz.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_viz.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable();

        path_en = this->declare_parameter("publish.path_en", true);
        scan_pub_en = this->declare_parameter("publish.scan_publish_en", true);
        dense_pub_en = this->declare_parameter("publish.dense_publish_en", true);
        scan_body_pub_en = this->declare_parameter("publish.scan_bodyframe_pub_en", true);
        NUM_MAX_ITERATIONS = this->declare_parameter("max_iteration", 4);
        map_file_path = this->declare_parameter("map_file_path", "");
        odom_topic = this->declare_parameter("common.odom_topic", "/scout_base_controller/odom");
        lid_topic = this->declare_parameter("common.lid_topic", "/livox/lidar");
        imu_topic = this->declare_parameter("common.imu_topic", "/livox/imu");
        time_sync_en = this->declare_parameter("common.time_sync_en", false);
        time_diff_lidar_to_imu = this->declare_parameter("common.time_offset_lidar_to_imu", 0.0);
        filter_size_surf_min = this->declare_parameter("filter_size_surf", 0.5);
        filter_size_map_min = this->declare_parameter("filter_size_map", 0.5);
        cube_len = this->declare_parameter("cube_side_length", 200.);
        DET_RANGE = this->declare_parameter("localization.det_range", 300.);
        fov_deg = this->declare_parameter("localization.fov_degree", 180.);
        gyr_cov = this->declare_parameter("localization.gyr_cov", 0.1);
        acc_cov = this->declare_parameter("localization.acc_cov", 0.1);
        b_gyr_cov = this->declare_parameter("localization.b_gyr_cov", 0.0001);
        b_acc_cov = this->declare_parameter("localization.b_acc_cov", 0.0001);
        kf_thres_ = this->declare_parameter("localization.keyframe_threshold", 0.5);
        dop_flag = this->declare_parameter("localization.dop_flag", false);
        p_pre->blind = this->declare_parameter("preprocess.blind", 0.01);
        p_pre->lidar_type = this->declare_parameter("preprocess.lidar_type", 0);
        p_pre->N_SCANS = this->declare_parameter("preprocess.scan_line", 16);
        p_pre->time_unit = this->declare_parameter("preprocess.timestamp_unit", 3);
        p_pre->SCAN_RATE = this->declare_parameter("preprocess.scan_rate", 10);
        point_filter_num = this->declare_parameter("point_filter_num", 2);
        p_pre->feature_enabled = this->declare_parameter("feature_extract_enable", false);
        extrinsic_est_en = this->declare_parameter("localization.extrinsic_est_en", true);
        extrinT = this->declare_parameter("localization.extrinsic_T", vector<double>());
        extrinR = this->declare_parameter("localization.extrinsic_R", vector<double>());
        extrin_g2o_T = this->declare_parameter("localization.extrinsic_g2o_T", vector<double>());
        extrin_g2o_R = this->declare_parameter("localization.extrinsic_g2o_R", vector<double>());
        odom_mode = this->declare_parameter("localization.odom_mode", 0);

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="odom";

        string map_directory = string(ROOT_DIR) + map_file_path;

        if (pcl::io::loadPCDFile(map_directory, *laserCloudMap) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Map file cannot open.");
            rclcpp::shutdown();
        }
        
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

        // Convert map from body frame to IMU frame for IEKF consistency
        RCLCPP_INFO(this->get_logger(), "Converting map from body frame to IMU frame...");

        downSizeFilterMap.setInputCloud(laserCloudMap);
        downSizeFilterMap.filter(*laserCloudMap);

        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, qos_viz, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, qos_viz, standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos_viz, imu_cbk);
        sub_initPose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos_viz, init_pose_cbk);
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, qos_viz,
            [this](const nav_msgs::msg::Odometry::ConstPtr &odom) {
                odom_callback(odom);
                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
            });
        
        sub_tf_static_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", qos_latched, tf_static_cbk);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos_viz);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", qos_viz);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", qos_viz);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", qos_latched);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", qos_viz);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", qos_viz);
        pubKeyFrame = this->create_publisher<fast_lio::msg::Frame> ("/key_frame", qos_viz);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(20.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserLocalizationNode::timer_callback, this));

        publish_map(pubLaserCloudMap_);

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserLocalizationNode()
    {}

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
            double match_start, solve_start;

            match_time = 0;
            
            p_imu->Process(Measures, kf, feats_undistort);
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
            down_dop = computeDOP(feats_down_body, Eigen::Vector3d(0,0,0));

            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);                    
                    ikdtree.Build(laserCloudMap->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(lidar_meas_cov, solve_H_time);  // 0.01 is dummy odom_R
            
            // solve_H_time 누적
            total_solve_H_time += solve_H_time;
            solve_H_time_count++;

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            M3D rot_base = state_point.rot.toRotationMatrix();
            Eigen::Quaterniond quat_base(rot_base);

            geoQuat.x = quat_base.x();
            geoQuat.y = quat_base.y();
            geoQuat.z = quat_base.z();
            geoQuat.w = quat_base.w();

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            if (odom_mode == 0) publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            if (initial_flag && dop_ratio > 1.4)
            {
                map_incremental();

                PointCloudXYZI::Ptr temp_worldMap(new PointCloudXYZI(*feats_undistort));
                for (size_t k = 0; k < feats_undistort->points.size(); k++)
                {
                    PointType p;
                    /* transform to world frame */
                    RGBpointLidarToWorld(&(feats_undistort->points[k]), &(temp_worldMap->points[k]));                    
                }
                *laserCloudMap += *temp_worldMap;                
                downSizeFilterMap.setInputCloud(laserCloudMap);
                downSizeFilterMap.filter(*laserCloudMap);
                cout<<"map updated."<<endl;
                publish_map(pubLaserCloudMap_);
            }
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (initial_flag)   publish_frame_body(pubLaserCloudFull_body_);
            publish_effect_world(pubLaserCloudEffect_);

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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initPose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserLocalizationNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();

    return 0;
}
