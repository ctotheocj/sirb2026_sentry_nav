// Copyright 2025 Pan — Apache-2.0
// DynObjFilter: ROS2 adaptation of M-Detector (HKU MARS Lab)

#pragma once
#ifndef M_DETECTOR__DYN_OBJ_FILTER_H_
#define M_DETECTOR__DYN_OBJ_FILTER_H_

#include <omp.h>
#include <mutex>
#include <cmath>
#include <array>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <execution>

#include <Eigen/Core>
#include <Eigen/LU>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "m-detector/types.h"
#include "m-detector/parallel_q.h"
#include "cluster_predict/voxel_cluster.h"

#define PI_MATH        (3.141593f)
#define MAX_2D_N       (564393)
#define MAX_1D         (1257)
#define MAX_1D_HALF    (449)
#define MAP_NUM        17
#define HASH_PRIM      19

enum dyn_obj_flg { STATIC, CASE1, CASE2, CASE3, SELF, UNCERTAIN, INVALID };

struct point_soph
{
    int          hor_ind{0}, ver_ind{0}, position{0};
    int          occu_times{0}, is_occu_times{0};
    Eigen::Vector3i occu_index, is_occu_index;
    double       time{-1};
    V3F          vec, occ_vec, is_occ_vec, cur_vec;
    M3D          rot;
    V3D          transl, glob, local, last_closest;
    dyn_obj_flg  dyn{STATIC};
    float        intensity{0};
    bool         is_distort{false};
    std::array<float, MAP_NUM>      last_depth_interps{};
    std::array<V3F,   HASH_PRIM>    last_vecs{};
    std::array<Eigen::Vector3i, HASH_PRIM> last_positions{};

    point_soph()
    {
        vec.setZero(); occ_vec.setZero(); is_occ_vec.setZero(); cur_vec.setZero();
        transl.setZero(); glob.setZero(); local.setZero(); last_closest.setZero();
        rot.setIdentity();
        occu_index  = -Eigen::Vector3i::Ones();
        is_occu_index = -Eigen::Vector3i::Ones();
        last_depth_interps.fill(0.f);
        for (auto & v : last_vecs) v.setZero();
        for (auto & p : last_positions) p.setZero();
    }

    point_soph(V3D & point, float & hor_res, float & ver_res) : point_soph()
    {
        vec(2) = float(point.norm());
        vec(0) = atan2f(float(point(1)), float(point(0)));
        vec(1) = atan2f(float(point(2)),
                        sqrtf(float(point(0))*float(point(0)) + float(point(1))*float(point(1))));
        hor_ind  = int(std::floor((vec(0) + PI_MATH) / hor_res));
        ver_ind  = int(std::floor((vec(1) + 0.5f * PI_MATH) / ver_res));
        position = hor_ind * MAX_1D_HALF + ver_ind;
    }

    void GetVec(V3D & point, float & hor_res, float & ver_res)
    {
        vec(2) = float(point.norm());
        vec(0) = atan2f(float(point(1)), float(point(0)));
        vec(1) = atan2f(float(point(2)),
                        sqrtf(float(point(0))*float(point(0)) + float(point(1))*float(point(1))));
        hor_ind  = int(std::floor((vec(0) + PI_MATH) / hor_res));
        ver_ind  = int(std::floor((vec(1) + 0.5f * PI_MATH) / ver_res));
        position = hor_ind * MAX_1D_HALF + ver_ind;
    }

    void reset()
    {
        occu_times = is_occu_times = 0;
        occu_index  = -Eigen::Vector3i::Ones();
        is_occu_index = -Eigen::Vector3i::Ones();
        occ_vec.setZero(); is_occ_vec.setZero(); last_closest.setZero();
        last_depth_interps.fill(0.f);
        for (auto & v : last_vecs) v.setZero();
        for (auto & p : last_positions) p.setZero();
        is_distort = false;
    }
};

using DepthMap2D = std::vector<std::vector<point_soph*>>;

class DepthMap
{
public:
    DepthMap2D       depth_map;
    double           time{0.};
    int              map_index{-1};
    M3D              project_R;
    V3D              project_T;
    float*           min_depth_static{nullptr};
    float*           min_depth_all{nullptr};
    float*           max_depth_all{nullptr};
    float*           max_depth_static{nullptr};
    int*             max_depth_index_all{nullptr};
    int*             min_depth_index_all{nullptr};
    std::vector<int> index_vector;

    using Ptr = std::shared_ptr<DepthMap>;

    DepthMap();
    DepthMap(M3D rot, V3D transl, double cur_time, int frame);
    ~DepthMap();
    void Reset(M3D rot, V3D transl, double cur_time, int frame);

private:
    void alloc();
};

class DynObjFilter
{
public:
    // --- tunable parameters (set via init()) ---
    float  depth_thr1{0.15f}, map_cons_depth_thr1{0.5f};
    float  map_cons_hor_thr1{0.02f}, map_cons_ver_thr1{0.01f};
    float  enter_min_thr1{2.0f}, enter_max_thr1{0.5f};
    float  map_cons_hor_dis1{0.2f}, map_cons_ver_dis1{0.2f};
    int    map_cons_hor_num1{0}, map_cons_ver_num1{0}, occluded_map_thr1{3};
    float  v_min_thr2{0.5f}, acc_thr2{1.0f}, v_min_thr3{0.5f}, acc_thr3{1.0f};
    float  map_cons_depth_thr2{0.15f}, map_cons_hor_thr2{0.02f}, map_cons_ver_thr2{0.01f};
    float  occ_depth_thr2{0.15f}, occ_hor_thr2{0.02f}, occ_ver_thr2{0.01f};
    float  depth_cons_depth_thr2{0.15f}, depth_cons_depth_max_thr2{0.15f};
    float  depth_cons_hor_thr2{0.02f}, depth_cons_ver_thr2{0.01f};
    float  depth_cons_depth_thr1{0.15f}, depth_cons_depth_max_thr1{1.0f};
    float  depth_cons_hor_thr1{0.02f}, depth_cons_ver_thr1{0.01f};
    int    map_cons_hor_num2{0}, map_cons_ver_num2{0};
    int    occ_hor_num2{0}, occ_ver_num2{0};
    int    depth_cons_hor_num1{0}, depth_cons_ver_num1{0};
    int    depth_cons_hor_num2{0}, depth_cons_ver_num2{0};
    int    occluded_times_thr2{3}, occluding_times_thr3{3};
    float  occ_depth_thr3{0.15f}, occ_hor_thr3{0.02f}, occ_ver_thr3{0.01f};
    float  map_cons_depth_thr3{0.15f}, map_cons_hor_thr3{0.02f}, map_cons_ver_thr3{0.01f};
    float  depth_cons_depth_thr3{0.15f}, depth_cons_depth_max_thr3{0.15f};
    float  depth_cons_hor_thr3{0.02f}, depth_cons_ver_thr3{0.01f};
    float  k_depth2{0.005f}, k_depth3{0.005f};
    int    map_cons_hor_num3{0}, map_cons_ver_num3{0};
    int    occ_hor_num3{0}, occ_ver_num3{0};
    int    depth_cons_hor_num3{0}, depth_cons_ver_num3{0};
    float  enlarge_z_thr1{0.05f}, enlarge_angle{2.f}, enlarge_depth{3.f};
    int    enlarge_distort{4}, checkneighbor_range{1};
    float  k_depth_min_thr1{0.f}, d_depth_min_thr1{50.f}, cutoff_value{0.f};
    float  k_depth_max_thr1{0.f}, d_depth_max_thr1{50.f};
    float  k_depth_max_thr2{0.f}, d_depth_max_thr2{50.f};
    float  k_depth_max_thr3{0.f}, d_depth_max_thr3{50.f};
    double frame_dur{0.1}, buffer_delay{0.1}, depth_map_dur{0.2};
    int    buffer_size{300000}, max_depth_map_num{5}, max_pixel_points{50};
    int    hor_num{MAX_1D}, ver_num{MAX_1D_HALF};
    float  hor_resolution_max{0.02f}, ver_resolution_max{0.02f};
    int    occu_time_th{3}, is_occu_time_th{3};
    float  voxel_filter_size{0.1f};
    float  blind_dis{0.3f};
    float  fov_up{2.f}, fov_down{-23.f}, fov_cut{-20.f};
    float  fov_left{180.f}, fov_right{-180.f};
    float  self_x_f{2.5f}, self_x_b{-1.5f}, self_y_l{1.6f}, self_y_r{-1.6f};
    bool   dyn_filter_en{true};
    bool   cluster_coupled{false}, cluster_future{false};
    bool   debug_en{false};
    bool   stop_object_detect{false};
    int    dataset{0};
    int    points_num_perframe{200000};
    int    case1_num{0}, case2_num{0}, case3_num{0};
    int    interp_hor_num{0}, interp_ver_num{0};
    bool   case1_interp_en{false}, case2_interp_en{false}, case3_interp_en{false};
    float  interp_hor_thr{0.01f}, interp_ver_thr{0.01f};
    float  interp_thr1{1.0f}, interp_thr2{1.0f}, interp_thr3{1.0f};
    float  interp_static_max{10.0f}, interp_start_depth1{30.f}, interp_kp1{0.1f}, interp_kd1{1.0f};
    float  interp_all_max{100.0f}, interp_start_depth2{30.f}, interp_kp2{0.1f}, interp_kd2{1.0f};
    std::string frame_id{"camera_init"};

    // --- output clouds ---
    PointCloudXYZI::Ptr laserCloudSteadObj;
    PointCloudXYZI::Ptr laserCloudDynObj;
    PointCloudXYZI::Ptr laserCloudDynObj_world;

    DynObjFilter();
    ~DynObjFilter() = default;

    void init(rclcpp::Node * node);
    void filter(PointCloudXYZI::Ptr feats_undistort,
                const M3D & rot_end, const V3D & pos_end, double scan_end_time);
    void publish_dyn(
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dyn,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_std,
        double scan_end_time);

private:
    std::deque<DepthMap::Ptr>        depth_map_list_;
    PARALLEL_Q<point_soph*>          buffer_;
    std::vector<point_soph*>         point_soph_pointers_;
    int                              cur_point_soph_pointers_{0};
    int                              max_pointers_num_{0};
    int                              map_index_{0};
    std::vector<int>                 dyn_tag_cluster_;
    std::vector<int>                 dyn_tag_origin_;
    DynObjCluster                    Cluster_;
    std::mutex                       mtx_case2_, mtx_case3_;
    int                              pixel_fov_up{0}, pixel_fov_down{0};
    int                              pixel_fov_cut{0}, pixel_fov_left{0}, pixel_fov_right{0};

    void  Points2Buffer(std::vector<point_soph*> & points, std::vector<int> & index_vector);
    void  Buffer2DepthMap(double cur_time);
    void  SphericalProjection(point_soph & p, int depth_index,
                              const M3D & rot, const V3D & transl,
                              point_soph & p_spherical);
    bool  Case1(point_soph & p);
    bool  Case1Enter(const point_soph & p, const DepthMap & map_info);
    bool  Case1FalseRejection(point_soph & p, const DepthMap & map_info);
    bool  Case1MapConsistencyCheck(point_soph & p, const DepthMap & map_info, bool interp);
    float DepthInterpolationStatic(point_soph & p, int midx, const DepthMap2D & dm);
    bool  Case2(point_soph & p);
    bool  Case2Enter(point_soph & p, const DepthMap & map_info);
    bool  Case2MapConsistencyCheck(point_soph & p, const DepthMap & map_info, bool interp);
    float DepthInterpolationAll(point_soph & p, int midx, const DepthMap2D & dm);
    bool  Case2DepthConsistencyCheck(const point_soph & p, const DepthMap & map_info);
    bool  Case2SearchPointOccludingP(point_soph & p, const DepthMap & map_info);
    bool  Case2IsOccluded(const point_soph & p, const point_soph & p_occ);
    bool  Case2VelCheck(float v1, float v2, double delta_t);
    bool  Case3(point_soph & p);
    bool  Case3Enter(point_soph & p, const DepthMap & map_info);
    bool  Case3MapConsistencyCheck(point_soph & p, const DepthMap & map_info, bool interp);
    bool  Case3SearchPointOccludedbyP(point_soph & p, const DepthMap & map_info);
    bool  Case3IsOccluding(const point_soph & p, const point_soph & p_occ);
    bool  Case3VelCheck(float v1, float v2, double delta_t);
    bool  Case3DepthConsistencyCheck(const point_soph & p, const DepthMap & map_info);
    bool  InvalidPointCheck(const V3D & body, int intensity);
    bool  SelfPointCheck(const V3D & body, dyn_obj_flg dyn);
    bool  CheckVerFoV(const point_soph & p, const DepthMap & map_info);
    void  CheckNeighbor(const point_soph & p, const DepthMap & map_info,
                        float & max_depth, float & min_depth);
};

#endif  // M_DETECTOR__DYN_OBJ_FILTER_H_
