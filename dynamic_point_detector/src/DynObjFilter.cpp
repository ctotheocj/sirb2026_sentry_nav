// Copyright 2025 Pan — Apache-2.0
// DynObjFilter: ROS2 adaptation of M-Detector (HKU MARS Lab)
// Algorithm: depth-map based dynamic point detection (Case1/2/3)

#include "m-detector/DynObjFilter.h"
#include <pcl_conversions/pcl_conversions.h>


void DepthMap::alloc()
{
    depth_map.assign(MAX_2D_N, std::vector<point_soph*>());
    min_depth_static  = new float[MAX_2D_N];
    min_depth_all     = new float[MAX_2D_N];
    max_depth_all     = new float[MAX_2D_N];
    max_depth_static  = new float[MAX_2D_N];
    max_depth_index_all = new int[MAX_2D_N];
    min_depth_index_all = new int[MAX_2D_N];
    std::fill_n(min_depth_static,  MAX_2D_N, 0.f);
    std::fill_n(min_depth_all,     MAX_2D_N, 0.f);
    std::fill_n(max_depth_all,     MAX_2D_N, 0.f);
    std::fill_n(max_depth_static,  MAX_2D_N, 0.f);
    std::fill_n(max_depth_index_all, MAX_2D_N, -1);
    std::fill_n(min_depth_index_all, MAX_2D_N, -1);
    index_vector.resize(MAX_2D_N);
    for (int i = 0; i < MAX_2D_N; ++i) index_vector[i] = i;
}

DepthMap::DepthMap()
{
    project_R.setIdentity();
    project_T.setZero();
    alloc();
}

DepthMap::DepthMap(M3D rot, V3D transl, double cur_time, int frame)
{
    project_R = rot; project_T = transl; time = cur_time; map_index = frame;
    alloc();
}

DepthMap::~DepthMap()
{
    delete[] min_depth_static;
    delete[] min_depth_all;
    delete[] max_depth_all;
    delete[] max_depth_static;
    delete[] max_depth_index_all;
    delete[] min_depth_index_all;
}

void DepthMap::Reset(M3D rot, V3D transl, double cur_time, int frame)
{
    project_R = rot; project_T = transl; time = cur_time; map_index = frame;
    std::for_each(std::execution::par, index_vector.begin(), index_vector.end(),
        [&](int i){ depth_map[i].clear(); });
    std::fill_n(min_depth_static,  MAX_2D_N, 0.f);
    std::fill_n(min_depth_all,     MAX_2D_N, 0.f);
    std::fill_n(max_depth_all,     MAX_2D_N, 0.f);
    std::fill_n(max_depth_static,  MAX_2D_N, 0.f);
    std::fill_n(max_depth_index_all, MAX_2D_N, -1);
    std::fill_n(min_depth_index_all, MAX_2D_N, -1);
}


DynObjFilter::DynObjFilter()
{
    laserCloudSteadObj    = std::make_shared<PointCloudXYZI>();
    laserCloudDynObj      = std::make_shared<PointCloudXYZI>();
    laserCloudDynObj_world= std::make_shared<PointCloudXYZI>();
}

void DynObjFilter::init(rclcpp::Node * node)
{
    node->declare_parameter("m_detector.depth_map_dur",      depth_map_dur);
    node->declare_parameter("m_detector.max_depth_map_num",  max_depth_map_num);
    node->declare_parameter("m_detector.hor_resolution",     double(hor_resolution_max));
    node->declare_parameter("m_detector.ver_resolution",     double(ver_resolution_max));
    node->declare_parameter("m_detector.depth_thr1",         double(depth_thr1));
    node->declare_parameter("m_detector.enter_min_thr1",     double(enter_min_thr1));
    node->declare_parameter("m_detector.enter_max_thr1",     double(enter_max_thr1));
    node->declare_parameter("m_detector.blind_dis",          double(blind_dis));
    node->declare_parameter("m_detector.fov_up",             double(fov_up));
    node->declare_parameter("m_detector.fov_down",           double(fov_down));
    node->declare_parameter("m_detector.cluster_coupled",    cluster_coupled);
    node->declare_parameter("m_detector.dyn_filter_en",      dyn_filter_en);
    node->declare_parameter("m_detector.frame_id",           frame_id);

    depth_map_dur     = node->get_parameter("m_detector.depth_map_dur").as_double();
    max_depth_map_num = node->get_parameter("m_detector.max_depth_map_num").as_int();
    hor_resolution_max= float(node->get_parameter("m_detector.hor_resolution").as_double());
    ver_resolution_max= float(node->get_parameter("m_detector.ver_resolution").as_double());
    depth_thr1        = float(node->get_parameter("m_detector.depth_thr1").as_double());
    enter_min_thr1    = float(node->get_parameter("m_detector.enter_min_thr1").as_double());
    enter_max_thr1    = float(node->get_parameter("m_detector.enter_max_thr1").as_double());
    blind_dis         = float(node->get_parameter("m_detector.blind_dis").as_double());
    fov_up            = float(node->get_parameter("m_detector.fov_up").as_double());
    fov_down          = float(node->get_parameter("m_detector.fov_down").as_double());
    cluster_coupled   = node->get_parameter("m_detector.cluster_coupled").as_bool();
    dyn_filter_en     = node->get_parameter("m_detector.dyn_filter_en").as_bool();
    frame_id          = node->get_parameter("m_detector.frame_id").as_string();

    map_cons_hor_num1 = int(std::ceil(map_cons_hor_thr1 / hor_resolution_max));
    map_cons_ver_num1 = int(std::ceil(map_cons_ver_thr1 / ver_resolution_max));
    interp_hor_num    = int(std::ceil(interp_hor_thr    / hor_resolution_max));
    interp_ver_num    = int(std::ceil(interp_ver_thr    / ver_resolution_max));
    map_cons_hor_num2 = int(std::ceil(map_cons_hor_thr2 / hor_resolution_max));
    map_cons_ver_num2 = int(std::ceil(map_cons_ver_thr2 / ver_resolution_max));
    occ_hor_num2      = int(std::ceil(occ_hor_thr2      / hor_resolution_max));
    occ_ver_num2      = int(std::ceil(occ_ver_thr2      / ver_resolution_max));
    depth_cons_hor_num2 = int(std::ceil(depth_cons_hor_thr2 / hor_resolution_max));
    depth_cons_ver_num2 = int(std::ceil(depth_cons_ver_thr2 / ver_resolution_max));
    map_cons_hor_num3 = int(std::ceil(map_cons_hor_thr3 / hor_resolution_max));
    map_cons_ver_num3 = int(std::ceil(map_cons_ver_thr3 / ver_resolution_max));
    occ_hor_num3      = int(std::ceil(occ_hor_thr3      / hor_resolution_max));
    occ_ver_num3      = int(std::ceil(occ_ver_thr3      / ver_resolution_max));
    depth_cons_hor_num3 = int(std::ceil(depth_cons_hor_thr3 / hor_resolution_max));
    depth_cons_ver_num3 = int(std::ceil(depth_cons_ver_thr3 / ver_resolution_max));

    pixel_fov_up    = int(std::floor((fov_up   * float(M_PI)/180.f + 0.5f*PI_MATH) / ver_resolution_max));
    pixel_fov_down  = int(std::floor((fov_down * float(M_PI)/180.f + 0.5f*PI_MATH) / ver_resolution_max));
    pixel_fov_cut   = int(std::floor((fov_cut  * float(M_PI)/180.f + 0.5f*PI_MATH) / ver_resolution_max));
    pixel_fov_left  = int(std::floor((fov_left * float(M_PI)/180.f + PI_MATH) / hor_resolution_max));
    pixel_fov_right = int(std::floor((fov_right* float(M_PI)/180.f + PI_MATH) / hor_resolution_max));

    max_pointers_num_ = int(std::round((max_depth_map_num * depth_map_dur + buffer_delay) / frame_dur)) + 1;
    point_soph_pointers_.reserve(max_pointers_num_);
    for (int i = 0; i < max_pointers_num_; i++)
        point_soph_pointers_.push_back(new point_soph[points_num_perframe]);
    buffer_.init(buffer_size);

    Cluster_.Init();
}


void DynObjFilter::SphericalProjection(
    point_soph & p, int depth_index,
    const M3D & rot, const V3D & transl, point_soph & ps)
{
    if (fabs(p.last_vecs.at(depth_index % HASH_PRIM)[2]) > 10E-5)
    {
        ps.vec      = p.last_vecs.at(depth_index % HASH_PRIM);
        ps.hor_ind  = p.last_positions.at(depth_index % HASH_PRIM)[0];
        ps.ver_ind  = p.last_positions.at(depth_index % HASH_PRIM)[1];
        ps.position = p.last_positions.at(depth_index % HASH_PRIM)[2];
    }
    else
    {
        V3D p_proj(rot * (p.glob - transl));
        ps.GetVec(p_proj, hor_resolution_max, ver_resolution_max);
        p.last_vecs.at(depth_index % HASH_PRIM) = ps.vec;
        p.last_positions.at(depth_index % HASH_PRIM)[0] = ps.hor_ind;
        p.last_positions.at(depth_index % HASH_PRIM)[1] = ps.ver_ind;
        p.last_positions.at(depth_index % HASH_PRIM)[2] = ps.position;
    }
}


void DynObjFilter::Points2Buffer(std::vector<point_soph*> & points, std::vector<int> & index_vector)
{
    int cur_tail = buffer_.tail;
    buffer_.push_parallel_prepare(points.size());
    std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int & i)
    {
        buffer_.push_parallel(points[i], cur_tail + i);
    });
}

void DynObjFilter::Buffer2DepthMap(double cur_time)
{
    int len = buffer_.size();
    for (int k = 0; k < len; k++)
    {
        point_soph * point = buffer_.front();
        if ((cur_time - point->time) >= buffer_delay - frame_dur / 2.0)
        {
            if (depth_map_list_.size() == 0)
            {
                if (int(depth_map_list_.size()) < max_depth_map_num)
                {
                    map_index_++;
                    depth_map_list_.push_back(std::make_shared<DepthMap>(
                        point->rot, point->transl, point->time, map_index_));
                }
                else
                {
                    buffer_.pop();
                    continue;
                }
            }
            else if ((point->time - depth_map_list_.back()->time) >= depth_map_dur - frame_dur / 2.0)
            {
                map_index_++;
                if (int(depth_map_list_.size()) == max_depth_map_num)
                {
                    depth_map_list_.front()->Reset(point->rot, point->transl, point->time, map_index_);
                    auto new_map = depth_map_list_.front();
                    depth_map_list_.pop_front();
                    depth_map_list_.push_back(new_map);
                }
                else
                {
                    depth_map_list_.push_back(std::make_shared<DepthMap>(
                        point->rot, point->transl, point->time, map_index_));
                }
            }
            SphericalProjection(*point, depth_map_list_.back()->map_index,
                                depth_map_list_.back()->project_R,
                                depth_map_list_.back()->project_T, *point);
            int pos = point->position;
            if (pos < 0 || pos >= MAX_2D_N) { buffer_.pop(); continue; }
            auto & dm = *depth_map_list_.back();
            if (int(dm.depth_map[pos].size()) < max_pixel_points)
            {
                dm.depth_map[pos].push_back(point);
                float d = point->vec(2);
                if (d > dm.max_depth_all[pos]) {
                    dm.max_depth_all[pos] = d;
                    dm.max_depth_index_all[pos] = int(dm.depth_map[pos].size()) - 1;
                }
                if (d < dm.min_depth_all[pos] || dm.min_depth_all[pos] < 1e-5f) {
                    dm.min_depth_all[pos] = d;
                    dm.min_depth_index_all[pos] = int(dm.depth_map[pos].size()) - 1;
                }
                if (point->dyn == STATIC) {
                    if (d < dm.min_depth_static[pos] || dm.min_depth_static[pos] < 1e-5f)
                        dm.min_depth_static[pos] = d;
                    if (d > dm.max_depth_static[pos])
                        dm.max_depth_static[pos] = d;
                }
            }
            buffer_.pop();
        }
        else
        {
            break;
        }
    }
}


bool DynObjFilter::InvalidPointCheck(const V3D & body, int /*intensity*/)
{
    if ((pow(body(0),2) + pow(body(1),2) + pow(body(2),2)) < blind_dis*blind_dis ||
        (dataset == 1 && fabs(body(0)) < 0.1 && fabs(body(1)) < 1.0 && fabs(body(2)) < 0.1))
        return true;
    return false;
}

bool DynObjFilter::SelfPointCheck(const V3D & body, dyn_obj_flg /*dyn*/)
{
    if (dataset == 0)
    {
        if ((body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 && body(2) > -0.65 && body(2) < -0.4) ||
            (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 && body(2) > -0.75 && body(2) < -0.40) ||
            (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 && body(2) > -0.8 && body(2) < -0.6) ||
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 && body(2) > -1.0 && body(2) < -0.9) ||
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 && body(2) > -1.0 && body(2) < -0.9))
            return true;
        return false;
    }
    return false;
}

bool DynObjFilter::CheckVerFoV(const point_soph & p, const DepthMap & map_info)
{
    bool ver_up = false, ver_down = false;
    for (int i = p.ver_ind; i >= pixel_fov_down; i--) {
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if (cur_pos >= 0 && cur_pos < MAX_2D_N && map_info.depth_map[cur_pos].size() > 0)
            { ver_down = true; break; }
    }
    for (int i = p.ver_ind; i <= pixel_fov_up; i++) {
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if (cur_pos >= 0 && cur_pos < MAX_2D_N && map_info.depth_map[cur_pos].size() > 0)
            { ver_up = true; break; }
    }
    return !(ver_up && ver_down);
}

void DynObjFilter::CheckNeighbor(
    const point_soph & p, const DepthMap & map_info,
    float & max_depth, float & min_depth)
{
    int n = checkneighbor_range;
    for (int i = -n; i <= n; i++) {
        for (int j = -n; j <= n; j++) {
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            if (cur_pos < MAX_2D_N && cur_pos >= 0 && map_info.depth_map[cur_pos].size() > 0) {
                float cur_max = map_info.max_depth_static[cur_pos];
                float cur_min = map_info.min_depth_static[cur_pos];
                if (min_depth > 10E-5) min_depth = std::min(cur_min, min_depth);
                else min_depth = cur_min;
                if (max_depth > 10E-5) max_depth = std::max(cur_max, max_depth);
                else max_depth = cur_max;
            }
        }
    }
}


bool DynObjFilter::Case1(point_soph & p)
{
    int depth_map_num = int(depth_map_list_.size());
    int occluded_map = depth_map_num;
    for (int i = depth_map_num - 1; i >= 0; i--)
    {
        SphericalProjection(p, depth_map_list_[i]->map_index,
            depth_map_list_[i]->project_R, depth_map_list_[i]->project_T, p);
        if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF ||
            p.vec(2) < 0.0f || p.position < 0 || p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            continue;
        }
        if (Case1Enter(p, *depth_map_list_[i]))
        {
            if (Case1FalseRejection(p, *depth_map_list_[i]))
                occluded_map -= 1;
        }
        else
        {
            occluded_map -= 1;
        }
        if (occluded_map < occluded_map_thr1) return false;
        if (occluded_map - i >= occluded_map_thr1) return true;
    }
    return occluded_map >= occluded_map_thr1;
}

bool DynObjFilter::Case1Enter(const point_soph & p, const DepthMap & map_info)
{
    float max_depth = 0, min_depth = 0;
    if (map_info.depth_map[p.position].size() > 0)
    {
        max_depth = map_info.max_depth_static[p.position];
        min_depth = map_info.min_depth_static[p.position];
    }
    else
    {
        if (p.ver_ind <= pixel_fov_up && p.ver_ind > pixel_fov_down &&
            p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right &&
            CheckVerFoV(p, map_info))
            CheckNeighbor(p, map_info, max_depth, min_depth);
    }
    float cur_min = std::max(cutoff_value, k_depth_min_thr1*(p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_max = std::max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;
    if (dataset == 0 && p.is_distort) { cur_min = enlarge_distort*cur_min; cur_max = enlarge_distort*cur_max; }
    if (p.vec(2) < min_depth - cur_max ||
        (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max) ||
        (stop_object_detect && min_depth < 10E-5 && max_depth < 10E-5 &&
         map_info.depth_map[p.position].size() > 0 && p.vec(2) < map_info.max_depth_all[p.position] + 1.0f))
    {
        case1_num++;
        return true;
    }
    return false;
}

bool DynObjFilter::Case1FalseRejection(point_soph & p, const DepthMap & map_info)
{
    return Case1MapConsistencyCheck(p, map_info, case1_interp_en);
}

bool DynObjFilter::Case1MapConsistencyCheck(point_soph & p, const DepthMap & map_info, bool interp)
{
    float hor_half = std::max(map_cons_hor_dis1 / std::max(p.vec(2), blind_dis), map_cons_hor_thr1);
    float ver_half = std::max(map_cons_ver_dis1 / std::max(p.vec(2), blind_dis), map_cons_ver_thr1);
    float cur_depth = std::max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + map_cons_depth_thr1;
    float cur_min   = std::max(cutoff_value, k_depth_min_thr1*(p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_max   = std::max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;
    if (dataset == 0 && p.is_distort) {
        cur_depth = enlarge_distort*cur_depth;
        cur_min   = enlarge_distort*cur_min;
        cur_max   = enlarge_distort*cur_max;
    }
    if (fabs(p.vec(1)) < enlarge_z_thr1 / 57.3f) {
        hor_half  = enlarge_angle * hor_half;
        ver_half  = enlarge_angle * ver_half;
        cur_depth = enlarge_depth * cur_depth;
    }
    int hn = int(std::ceil(hor_half / hor_resolution_max));
    int vn = int(std::ceil(ver_half / ver_resolution_max));
    for (int ih = -hn; ih <= hn; ih++) {
        for (int iv = -vn; iv <= vn; iv++) {
            int pos_new = ((p.hor_ind + ih) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + iv) % MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            if (map_info.max_depth_static[pos_new] < p.vec(2) - cur_min ||
                map_info.min_depth_static[pos_new] > p.vec(2) + cur_max) continue;
            for (const auto * pt : map_info.depth_map[pos_new]) {
                if (pt->dyn == STATIC &&
                    (fabs(p.vec(2)-pt->vec(2)) < cur_depth ||
                     ((p.vec(2)-pt->vec(2)) > cur_depth && (p.vec(2)-pt->vec(2)) < cur_min)) &&
                    fabs(p.vec(0)-pt->vec(0)) < hor_half &&
                    fabs(p.vec(1)-pt->vec(1)) < ver_half)
                    return true;
            }
        }
    }
    if (interp && (p.local(0) < self_x_b || p.local(0) > self_x_f ||
                   p.local(1) > self_y_l  || p.local(1) < self_y_r)) {
        float depth_s = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        float cur_interp = interp_thr1;
        if (p.vec(2) > interp_start_depth1)
            cur_interp += (p.vec(2) - interp_start_depth1) * interp_kp1 + interp_kd1;
        if (fabs(depth_s + 1) < 10E-5 || fabs(depth_s + 2) < 10E-5) return false;
        if (fabs(depth_s - p.vec(2)) < cur_interp) return true;
    }
    return false;
}

float DynObjFilter::DepthInterpolationStatic(point_soph & p, int map_index, const DepthMap2D & depth_map)
{
    int idx = map_index - depth_map_list_.front()->map_index;
    if (idx >= 0 && idx < MAP_NUM && fabs(p.last_depth_interps.at(idx)) > 10E-4)
        return p.last_depth_interps.at(idx);
    V3F p_1 = V3F::Zero(), p_2 = V3F::Zero(), p_3 = V3F::Zero();
    std::vector<V3F> p_neighbors;
    int all_num = 0, static_num = 0;
    for (int ih = -interp_hor_num; ih <= interp_hor_num; ih++) {
        for (int iv = -interp_ver_num; iv <= interp_ver_num; iv++) {
            int pos_new = ((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF + ((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            for (const auto * pt : depth_map[pos_new]) {
                if (fabs(pt->time - p.time) < frame_dur) continue;
                float hm = pt->vec(0)-p.vec(0), vm = pt->vec(1)-p.vec(1);
                if (fabs(hm) < interp_hor_thr && fabs(vm) < interp_ver_thr) {
                    all_num++;
                    if (pt->dyn == STATIC) {
                        static_num++;
                        if ((pt->vec(2)-p.vec(2)) <= interp_static_max && (p.vec(2)-pt->vec(2)) < 5.0f) {}
                        p_neighbors.push_back(pt->vec);
                        if (p_1(2) < 0.000001f || fabs(hm)+fabs(vm) < fabs(p_1(0)-p.vec(0))+fabs(p_1(1)-p.vec(1)))
                            p_1 = pt->vec;
                    }
                }
            }
        }
    }
    if (p_1(2) < 10E-5) { if (idx>=0&&idx<MAP_NUM) p.last_depth_interps.at(idx)=-1; return -1; }
    int cur_size = int(p_neighbors.size());
    for (int ti = 0; ti < cur_size-2; ti++) {
        p_1 = p_neighbors[ti]; p_2 = V3F::Zero(); p_3 = V3F::Zero();
        float min_fabs = 2*(interp_hor_thr+interp_ver_thr);
        float x = p.vec(0)-p_1(0), y = p.vec(1)-p_1(1), alpha=0, beta=0;
        for (int i = ti+1; i < cur_size-1; i++) {
            if (fabs(p_neighbors[i](0)-p.vec(0))+fabs(p_neighbors[i](1)-p.vec(1)) < min_fabs) {
                p_2 = p_neighbors[i];
                float sf = fabs(p_neighbors[i](0)-p.vec(0))+fabs(p_neighbors[i](1)-p.vec(1));
                if (sf >= min_fabs) continue;
                for (int ii = i+1; ii < cur_size; ii++) {
                    float cf = fabs(p_neighbors[i](0)-p.vec(0))+fabs(p_neighbors[i](1)-p.vec(1))+
                               fabs(p_neighbors[ii](0)-p.vec(0))+fabs(p_neighbors[ii](1)-p.vec(1));
                    if (cf < min_fabs) {
                        float x1=p_neighbors[i](0)-p_1(0), x2=p_neighbors[ii](0)-p_1(0);
                        float y1=p_neighbors[i](1)-p_1(1), y2=p_neighbors[ii](1)-p_1(1);
                        float lower=x1*y2-x2*y1;
                        if (fabs(lower)>10E-5) {
                            alpha=(x*y2-y*x2)/lower; beta=-(x*y1-y*x1)/lower;
                            if (alpha>0&&alpha<1&&beta>0&&beta<1&&(alpha+beta)>0&&(alpha+beta)<1)
                                { p_3=p_neighbors[ii]; min_fabs=cf; }
                        }
                    }
                }
            }
        }
        if (p_2(2)<10E-5||p_3(2)<10E-5) continue;
        float dc=(1-alpha-beta)*p_1(2)+alpha*p_2(2)+beta*p_3(2);
        if (idx>=0&&idx<MAP_NUM) p.last_depth_interps.at(idx)=dc;
        return dc;
    }
    if (idx>=0&&idx<MAP_NUM) p.last_depth_interps.at(idx)=-2;
    return -2;
}

// old Case1 stub removed — new Case1 is above

bool DynObjFilter::Case2(point_soph & p)
{
    if (dataset == 0 && p.is_distort) return false;
    int first_i = int(depth_map_list_.size()) - 1;
    if (first_i < 0) return false;
    point_soph p_spherical = p;
    SphericalProjection(p, depth_map_list_[first_i]->map_index,
        depth_map_list_[first_i]->project_R, depth_map_list_[first_i]->project_T, p_spherical);
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF ||
        p_spherical.vec(2) < 0.f || p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
        { p.dyn = INVALID; return false; }
    int cur_occ_times = 0;
    if (Case2Enter(p_spherical, *depth_map_list_[first_i]))
    {
        if (!Case2MapConsistencyCheck(p_spherical, *depth_map_list_[first_i], case2_interp_en))
        {
            bool map_cons = true;
            for (int ih = -occ_hor_num2; ih <= occ_hor_num2 && map_cons; ih++) {
                for (int iv = -occ_ver_num2; iv <= occ_ver_num2 && map_cons; iv++) {
                    int pos_new = ((p_spherical.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p_spherical.ver_ind+iv)%MAX_1D_HALF);
                    if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
                    if (depth_map_list_[first_i]->min_depth_all[pos_new] > p_spherical.vec(2)) continue;
                    for (int k = 0; k < int(depth_map_list_[first_i]->depth_map[pos_new].size()) && map_cons; k++) {
                        const point_soph * p_occ = depth_map_list_[first_i]->depth_map[pos_new][k];
                        if (Case2IsOccluded(p_spherical, *p_occ) && Case2DepthConsistencyCheck(*p_occ, *depth_map_list_[first_i])) {
                            cur_occ_times = 1;
                            if (cur_occ_times >= occluded_times_thr2) break;
                            double ti = (p_occ->time + p.time) / 2;
                            float vi = (p_spherical.vec(2) - p_occ->vec(2)) / float(p.time - p_occ->time);
                            p.occu_index[0] = depth_map_list_[first_i]->map_index;
                            p.occu_index[1] = pos_new; p.occu_index[2] = k;
                            p.occ_vec = p_spherical.vec; p.occu_times = cur_occ_times;
                            point_soph p1 = *depth_map_list_[first_i]->depth_map[pos_new][k];
                            int i = int(depth_map_list_.size()) - 2;
                            while (i >= 0) {
                                if (p1.occu_index[0] == -1 || p1.occu_index[0] < depth_map_list_.front()->map_index) {
                                    SphericalProjection(p1, depth_map_list_[i]->map_index,
                                        depth_map_list_[i]->project_R, depth_map_list_[i]->project_T, p1);
                                    if (Case2SearchPointOccludingP(p1, *depth_map_list_[i])) p1.occ_vec = p1.vec;
                                    else break;
                                }
                                i = p1.occu_index[0] - depth_map_list_.front()->map_index;
                                if (i < 0 || i >= int(depth_map_list_.size())) break;
                                point_soph * p2 = depth_map_list_[i]->depth_map[p1.occu_index[1]][p1.occu_index[2]];
                                SphericalProjection(p, depth_map_list_[i]->map_index,
                                    depth_map_list_[i]->project_R, depth_map_list_[i]->project_T, p);
                                if (Case2MapConsistencyCheck(p, *depth_map_list_[i], case2_interp_en)) { map_cons=false; break; }
                                float vc = (p1.occ_vec(2) - p2->vec(2)) / float(p1.time - p2->time);
                                double tc = (p2->time + p1.time) / 2;
                                if (Case2IsOccluded(p, *p2) && Case2DepthConsistencyCheck(*p2, *depth_map_list_[i]) && Case2VelCheck(vi, vc, ti-tc)) {
                                    cur_occ_times++;
                                    if (cur_occ_times >= occluded_times_thr2) { p.occu_times=cur_occ_times; return true; }
                                    p1 = *p2; vi = vc; ti = tc;
                                } else break;
                                i--;
                            }
                        }
                        if (cur_occ_times >= occluded_times_thr2) break;
                    }
                }
            }
        }
    }
    if (cur_occ_times >= occluded_times_thr2) { p.occu_times = cur_occ_times; return true; }
    return false;
}

bool DynObjFilter::Case2Enter(point_soph & p, const DepthMap & map_info)
{
    if (p.dyn != STATIC) return false;
    float max_depth = 0;
    float depth_thr2_final = std::max(cutoff_value, k_depth_max_thr2*(p.vec(2)-d_depth_max_thr2)) + occ_depth_thr2;
    if (map_info.depth_map[p.position].size() > 0) {
        const point_soph * mp = map_info.depth_map[p.position][map_info.max_depth_index_all[p.position]];
        max_depth = mp->vec(2);
        depth_thr2_final = std::min(depth_thr2_final, v_min_thr2 * float(p.time - mp->time));
    }
    if (p.vec(2) > max_depth + depth_thr2_final) { case2_num++; return true; }
    return false;
}

bool DynObjFilter::Case2MapConsistencyCheck(point_soph & p, const DepthMap & map_info, bool interp)
{
    float cur_depth = std::max(cutoff_value, k_depth_max_thr2*(p.vec(2)-d_depth_max_thr2)) + map_cons_depth_thr2;
    for (int ih = -map_cons_hor_num2; ih <= map_cons_hor_num2; ih++) {
        for (int iv = -map_cons_ver_num2; iv <= map_cons_ver_num2; iv++) {
            int pos_new = ((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            if (map_info.max_depth_all[pos_new] > p.vec(2)+cur_depth && map_info.min_depth_all[pos_new] < p.vec(2)-cur_depth) continue;
            for (const auto * pt : map_info.depth_map[pos_new]) {
                if (pt->dyn == STATIC && fabs(p.time-pt->time) > frame_dur &&
                    fabs(p.vec(2)-pt->vec(2)) < cur_depth &&
                    fabs(p.vec(0)-pt->vec(0)) < map_cons_hor_thr2 &&
                    fabs(p.vec(1)-pt->vec(1)) < map_cons_ver_thr2) return true;
            }
        }
    }
    if (interp && (p.local(0)<self_x_b||p.local(0)>self_x_f||p.local(1)>self_y_l||p.local(1)<self_y_r)) {
        float cur_interp = interp_thr2 * float(depth_map_list_.back()->map_index - map_info.map_index + 1);
        float da = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        return fabs(p.vec(2) - da) < cur_interp;
    }
    return false;
}

bool DynObjFilter::Case2SearchPointOccludingP(point_soph & p, const DepthMap & map_info)
{
    for (int ih = -occ_hor_num2; ih <= occ_hor_num2; ih++) {
        for (int iv = -occ_ver_num2; iv <= occ_ver_num2; iv++) {
            int pos_new = ((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            if (map_info.min_depth_all[pos_new] > p.vec(2)) continue;
            for (int j = 0; j < int(map_info.depth_map[pos_new].size()); j++) {
                const point_soph * pc = map_info.depth_map[pos_new][j];
                if (Case2IsOccluded(p, *pc) && Case2DepthConsistencyCheck(*pc, map_info)) {
                    p.occu_index[0]=map_info.map_index; p.occu_index[1]=pos_new; p.occu_index[2]=j;
                    p.occ_vec = p.vec; return true;
                }
            }
        }
    }
    return false;
}

bool DynObjFilter::Case2IsOccluded(const point_soph & p, const point_soph & p_occ)
{
    if ((dataset==0&&p_occ.is_distort)||(dataset==0&&p.is_distort)||p_occ.dyn==INVALID) return false;
    if ((p.local(0)>self_x_b&&p.local(0)<self_x_f&&p.local(1)<self_y_l&&p.local(1)>self_y_r)||
        (p_occ.local(0)>self_x_b&&p_occ.local(0)<self_x_f&&p_occ.local(1)<self_y_l&&p_occ.local(1)>self_y_r)) return false;
    float delta_t = float(p.time - p_occ.time);
    if (delta_t > 0) {
        float thr = std::min(std::max(cutoff_value, k_depth_max_thr2*(p.vec(2)-d_depth_max_thr2))+occ_depth_thr2, v_min_thr2*delta_t);
        if (p.vec(2) > p_occ.vec(2)+thr && fabs(p.vec(0)-p_occ.vec(0))<occ_hor_thr2 && fabs(p.vec(1)-p_occ.vec(1))<occ_ver_thr2) return true;
    }
    return false;
}

bool DynObjFilter::Case2DepthConsistencyCheck(const point_soph & p, const DepthMap & map_info)
{
    float all_minus=0; int num=0, smaller_num=0, all_num=0, greater_num=0;
    for (int ih=-depth_cons_hor_num2; ih<=depth_cons_hor_num2; ih++) {
        for (int iv=-depth_cons_ver_num2; iv<=depth_cons_ver_num2; iv++) {
            int pos_new=((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new<0||pos_new>=MAX_2D_N) continue;
            for (const auto * pt : map_info.depth_map[pos_new]) {
                if (fabs(pt->time-p.time)<frame_dur && fabs(pt->vec(0)-p.vec(0))<depth_cons_hor_thr2 && fabs(pt->vec(1)-p.vec(1))<depth_cons_ver_thr2) {
                    all_num++;
                    if (pt->dyn==STATIC) {
                        float cm=p.vec(2)-pt->vec(2);
                        if (fabs(cm)<depth_cons_depth_max_thr2) { num++; all_minus+=fabs(pt->vec(2)-p.vec(2)); }
                        else if (cm>0) smaller_num++; else greater_num++;
                    }
                }
            }
        }
    }
    if (all_num>0) {
        if (num>1 && all_minus/(num-1) > std::max(depth_cons_depth_thr2, k_depth2*p.vec(2))) return false;
        return greater_num==0 || smaller_num==0;
    }
    return false;
}

bool DynObjFilter::Case2VelCheck(float v1, float v2, double delta_t)
{
    return fabs(v1-v2) < float(delta_t)*acc_thr2;
}

float DynObjFilter::DepthInterpolationAll(point_soph & p, int map_index, const DepthMap2D & depth_map)
{
    V3F p_1=V3F::Zero(); std::vector<V3F> p_neighbors; int all_num=0;
    for (int ih=-interp_hor_num; ih<=interp_hor_num; ih++) {
        for (int iv=-interp_ver_num; iv<=interp_ver_num; iv++) {
            int pos_new=((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new<0||pos_new>=MAX_2D_N) continue;
            for (const auto * pt : depth_map[pos_new]) {
                if (fabs(pt->time-p.time)<frame_dur) continue;
                float hm=pt->vec(0)-p.vec(0), vm=pt->vec(1)-p.vec(1);
                if (fabs(hm)<interp_hor_thr&&fabs(vm)<interp_ver_thr) {
                    all_num++; p_neighbors.push_back(pt->vec);
                    if (p_1(2)<0.000001f||fabs(hm)+fabs(vm)<fabs(p_1(0)-p.vec(0))+fabs(p_1(1)-p.vec(1))) p_1=pt->vec;
                }
            }
        }
    }
    int cur_size=int(p_neighbors.size());
    if (p_1(2)<10E-5||cur_size<3) return -1;
    for (int ti=0; ti<cur_size-2; ti++) {
        p_1=p_neighbors[ti]; V3F p_2=V3F::Zero(), p_3=V3F::Zero();
        float min_fabs=2*(interp_hor_thr+interp_ver_thr), x=p.vec(0)-p_1(0), y=p.vec(1)-p_1(1), alpha=0, beta=0;
        for (int i=ti+1; i<cur_size-1; i++) {
            if (fabs(p_neighbors[i](0)-p.vec(0))+fabs(p_neighbors[i](1)-p.vec(1))<min_fabs) {
                p_2=p_neighbors[i];
                float sf=fabs(p_neighbors[i](0)-p.vec(0))+fabs(p_neighbors[i](1)-p.vec(1));
                if (sf>=min_fabs) continue;
                for (int ii=i+1; ii<cur_size; ii++) {
                    float cf=fabs(p_neighbors[i](0)-p.vec(0))+fabs(p_neighbors[i](1)-p.vec(1))+fabs(p_neighbors[ii](0)-p.vec(0))+fabs(p_neighbors[ii](1)-p.vec(1));
                    if (cf<min_fabs) {
                        float x1=p_neighbors[i](0)-p_1(0),x2=p_neighbors[ii](0)-p_1(0),y1=p_neighbors[i](1)-p_1(1),y2=p_neighbors[ii](1)-p_1(1);
                        float lower=x1*y2-x2*y1;
                        if (fabs(lower)>10E-5) {
                            alpha=(x*y2-y*x2)/lower; beta=-(x*y1-y*x1)/lower;
                            if (alpha>0&&alpha<1&&beta>0&&beta<1&&(alpha+beta)>0&&(alpha+beta)<1) { p_3=p_neighbors[ii]; min_fabs=cf; }
                        }
                    }
                }
            }
        }
        if (p_2(2)<10E-5||p_3(2)<10E-5) continue;
        return (1-alpha-beta)*p_1(2)+alpha*p_2(2)+beta*p_3(2);
    }
    return -2;
}

// old Case2 stub removed — new Case2 is above

bool DynObjFilter::Case3(point_soph & p)
{
    if (dataset == 0 && p.is_distort) return false;
    int first_i = int(depth_map_list_.size()) - 1;
    if (first_i < 0) return false;
    point_soph p_spherical = p;
    SphericalProjection(p, depth_map_list_[first_i]->map_index,
        depth_map_list_[first_i]->project_R, depth_map_list_[first_i]->project_T, p_spherical);
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF ||
        p_spherical.vec(2) < 0.f || p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
        { p.dyn = INVALID; return false; }
    int cur_occ_times = 0;
    if (Case3Enter(p_spherical, *depth_map_list_[first_i]))
    {
        if (!Case3MapConsistencyCheck(p_spherical, *depth_map_list_[first_i], case3_interp_en))
        {
            bool map_cons = true;
            for (int ih = -occ_hor_num3; ih <= occ_hor_num3 && map_cons; ih++) {
                for (int iv = -occ_ver_num3; iv <= occ_ver_num3 && map_cons; iv++) {
                    int pos_new = ((p_spherical.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p_spherical.ver_ind+iv)%MAX_1D_HALF);
                    if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
                    if (depth_map_list_[first_i]->max_depth_all[pos_new] < p_spherical.vec(2)) continue;
                    for (int k = 0; k < int(depth_map_list_[first_i]->depth_map[pos_new].size()) && map_cons; k++) {
                        const point_soph * p_occ = depth_map_list_[first_i]->depth_map[pos_new][k];
                        if (Case3IsOccluding(p_spherical, *p_occ) && Case3DepthConsistencyCheck(*p_occ, *depth_map_list_[first_i])) {
                            cur_occ_times = 1;
                            double ti = (p_occ->time + p.time) / 2;
                            float vi = (p_occ->vec(2) - p_spherical.vec(2)) / float(p.time - p_occ->time);
                            p.is_occu_index[0]=depth_map_list_[first_i]->map_index;
                            p.is_occu_index[1]=pos_new; p.is_occu_index[2]=k;
                            p.is_occ_vec=p_spherical.vec; p.is_occu_times=cur_occ_times;
                            point_soph p1 = *depth_map_list_[first_i]->depth_map[pos_new][k];
                            int i = int(depth_map_list_.size()) - 2;
                            while (i >= 0) {
                                if (p1.is_occu_index[0]==-1||p1.is_occu_index[0]<depth_map_list_.front()->map_index) {
                                    SphericalProjection(p1, depth_map_list_[i]->map_index,
                                        depth_map_list_[i]->project_R, depth_map_list_[i]->project_T, p1);
                                    if (Case3SearchPointOccludedbyP(p1, *depth_map_list_[i])) p1.is_occ_vec=p1.vec;
                                    else break;
                                }
                                i = p1.is_occu_index[0] - depth_map_list_.front()->map_index;
                                if (i < 0 || i >= int(depth_map_list_.size())) break;
                                point_soph * p2 = depth_map_list_[i]->depth_map[p1.is_occu_index[1]][p1.is_occu_index[2]];
                                SphericalProjection(p, depth_map_list_[i]->map_index,
                                    depth_map_list_[i]->project_R, depth_map_list_[i]->project_T, p);
                                if (Case3MapConsistencyCheck(p, *depth_map_list_[i], case3_interp_en)) { map_cons=false; break; }
                                float vc = -(p1.is_occ_vec(2)-p2->vec(2))/float(p1.time-p2->time);
                                double tc = (p2->time+p1.time)/2;
                                if (Case3IsOccluding(p,*p2)&&Case3DepthConsistencyCheck(*p2,*depth_map_list_[i])&&Case3VelCheck(vi,vc,ti-tc)) {
                                    cur_occ_times++;
                                    if (cur_occ_times>=occluding_times_thr3) { p.is_occu_times=cur_occ_times; return true; }
                                    p1=*p2; vi=vc; ti=tc;
                                } else break;
                                i--;
                            }
                        }
                        if (cur_occ_times>=occluding_times_thr3) break;
                    }
                }
            }
        }
    }
    if (cur_occ_times>=occluding_times_thr3) { p.is_occu_times=cur_occ_times; return true; }
    return false;
}

bool DynObjFilter::Case3Enter(point_soph & p, const DepthMap & map_info)
{
    if (p.dyn != STATIC) return false;
    float min_depth = 0;
    float thr = std::max(cutoff_value, k_depth_max_thr3*(p.vec(2)-d_depth_max_thr3)) + occ_depth_thr3;
    if (map_info.depth_map[p.position].size() > 0) {
        const point_soph * mp = map_info.depth_map[p.position][map_info.min_depth_index_all[p.position]];
        min_depth = mp->vec(2);
        thr = std::min(thr, v_min_thr3 * float(p.time - mp->time));
    }
    if (dataset==0&&p.is_distort) thr = enlarge_distort*thr;
    if (p.vec(2) < min_depth - thr) { case3_num++; return true; }
    return false;
}

bool DynObjFilter::Case3MapConsistencyCheck(point_soph & p, const DepthMap & map_info, bool interp)
{
    float cur_depth = std::max(cutoff_value, k_depth_max_thr3*(p.vec(2)-d_depth_max_thr3)) + map_cons_depth_thr3;
    for (int ih=-map_cons_hor_num3; ih<=map_cons_hor_num3; ih++) {
        for (int iv=-map_cons_ver_num3; iv<=map_cons_ver_num3; iv++) {
            int pos_new=((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new<0||pos_new>=MAX_2D_N) continue;
            if (map_info.max_depth_all[pos_new]>p.vec(2)+cur_depth && map_info.min_depth_all[pos_new]<p.vec(2)-cur_depth) continue;
            for (const auto * pt : map_info.depth_map[pos_new]) {
                if (pt->dyn==STATIC && fabs(p.time-pt->time)>frame_dur &&
                    (pt->vec(2)-p.vec(2))<cur_depth &&
                    fabs(p.vec(0)-pt->vec(0))<map_cons_hor_thr3 &&
                    fabs(p.vec(1)-pt->vec(1))<map_cons_ver_thr3) return true;
            }
        }
    }
    if (interp && (p.local(0)<self_x_b||p.local(0)>self_x_f||p.local(1)>self_y_l||p.local(1)<self_y_r)) {
        float cur_interp = interp_thr3 * float(depth_map_list_.back()->map_index - map_info.map_index + 1);
        float da = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        return fabs(p.vec(2)-da) < cur_interp;
    }
    return false;
}

bool DynObjFilter::Case3SearchPointOccludedbyP(point_soph & p, const DepthMap & map_info)
{
    for (int ih=-occ_hor_num3; ih<=occ_hor_num3; ih++) {
        for (int iv=-occ_ver_num3; iv<=occ_ver_num3; iv++) {
            int pos_new=((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new<0||pos_new>=MAX_2D_N) continue;
            if (map_info.min_depth_all[pos_new]>p.vec(2)) continue;
            for (int j=0; j<int(map_info.depth_map[pos_new].size()); j++) {
                const point_soph * pc = map_info.depth_map[pos_new][j];
                if (Case3IsOccluding(p,*pc)&&Case3DepthConsistencyCheck(*pc,map_info)) {
                    p.is_occu_index[0]=map_info.map_index; p.is_occu_index[1]=pos_new; p.is_occu_index[2]=j;
                    p.occ_vec=p.vec; return true;
                }
            }
        }
    }
    return false;
}

bool DynObjFilter::Case3IsOccluding(const point_soph & p, const point_soph & p_occ)
{
    if ((dataset==0&&p_occ.is_distort)||(dataset==0&&p.is_distort)||p_occ.dyn==INVALID) return false;
    if ((p.local(0)>self_x_b&&p.local(0)<self_x_f&&p.local(1)<self_y_l&&p.local(1)>self_y_r)||
        (p_occ.local(0)>self_x_b&&p_occ.local(0)<self_x_f&&p_occ.local(1)<self_y_l&&p_occ.local(1)>self_y_r)) return false;
    float delta_t = float(p.time - p_occ.time);
    if (delta_t > 0) {
        float thr = std::min(std::max(cutoff_value, k_depth_max_thr3*(p.vec(2)-d_depth_max_thr3))+map_cons_depth_thr3, v_min_thr3*delta_t);
        if (dataset==0&&p.is_distort) thr=enlarge_distort*thr;
        if (p_occ.vec(2)>p.vec(2)+thr && fabs(p.vec(0)-p_occ.vec(0))<occ_hor_thr3 && fabs(p.vec(1)-p_occ.vec(1))<occ_ver_thr3) return true;
    }
    return false;
}

bool DynObjFilter::Case3DepthConsistencyCheck(const point_soph & p, const DepthMap & map_info)
{
    float all_minus=0; int num=0, smaller_num=0, all_num=0, greater_num=0;
    for (int ih=-depth_cons_hor_num3; ih<=depth_cons_hor_num3; ih++) {
        for (int iv=-depth_cons_ver_num3; iv<=depth_cons_ver_num3; iv++) {
            int pos_new=((p.hor_ind+ih)%MAX_1D)*MAX_1D_HALF+((p.ver_ind+iv)%MAX_1D_HALF);
            if (pos_new<0||pos_new>=MAX_2D_N) continue;
            for (const auto * pt : map_info.depth_map[pos_new]) {
                if (fabs(pt->time-p.time)<frame_dur && fabs(pt->vec(0)-p.vec(0))<depth_cons_hor_thr3 && fabs(pt->vec(1)-p.vec(1))<depth_cons_ver_thr3) {
                    all_num++;
                    if (pt->dyn==STATIC) {
                        float cm=p.vec(2)-pt->vec(2);
                        if (fabs(cm)<depth_cons_depth_max_thr3) { num++; all_minus+=fabs(pt->vec(2)-p.vec(2)); }
                        else if (cm>0) smaller_num++; else greater_num++;
                    }
                }
            }
        }
    }
    if (all_num>0) {
        if (num>1 && all_minus/(num-1)>std::max(depth_cons_depth_thr3, k_depth3*p.vec(2))) return false;
        return greater_num==0||smaller_num==0;
    }
    return false;
}

bool DynObjFilter::Case3VelCheck(float v1, float v2, double delta_t)
{
    return fabs(v1-v2) < float(delta_t)*acc_thr3;
}


void DynObjFilter::filter(
    PointCloudXYZI::Ptr feats_undistort,
    const M3D & rot_end, const V3D & pos_end, double scan_end_time)
{
    const int size = int(feats_undistort->size());
    if (size == 0) return;

    laserCloudSteadObj->clear();
    laserCloudDynObj->clear();
    laserCloudDynObj_world->clear();

    dyn_tag_origin_.assign(size, 0);
    dyn_tag_cluster_.assign(size, 0);

    point_soph * p = point_soph_pointers_[cur_point_soph_pointers_];

    std::vector<int> index(size);
    for (int i = 0; i < size; ++i) index[i] = i;

    std::vector<point_soph*> cur_pts(size);
    std::for_each(std::execution::par, index.begin(), index.end(), [&](int i) {
        p[i].reset();
        V3D p_body(feats_undistort->points[i].x,
                   feats_undistort->points[i].y,
                   feats_undistort->points[i].z);
        int intensity = feats_undistort->points[i].curvature;
        p[i].glob   = rot_end * p_body + pos_end;
        p[i].dyn    = STATIC;
        p[i].rot    = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time   = scan_end_time;
        p[i].local  = p_body;
        p[i].intensity = feats_undistort->points[i].intensity;
        if (dataset == 0 && fabs(intensity - 666) < 10E-4)
            p[i].is_distort = true;

        if (InvalidPointCheck(p_body, intensity)) {
            p[i].dyn = INVALID; dyn_tag_origin_[i] = 0; dyn_tag_cluster_[i] = -1;
        } else if (SelfPointCheck(p_body, p[i].dyn)) {
            p[i].dyn = INVALID; dyn_tag_origin_[i] = 0;
        } else if (Case1(p[i])) {
            p[i].dyn = CASE1; dyn_tag_origin_[i] = 1;
        } else if (Case2(p[i])) {
            p[i].dyn = CASE2; dyn_tag_origin_[i] = 1;
        } else if (Case3(p[i])) {
            p[i].dyn = CASE3; dyn_tag_origin_[i] = 1;
        } else {
            dyn_tag_origin_[i] = 0;
        }
        cur_pts[i] = &p[i];
    });

    // Build output clouds
    for (int i = 0; i < size; ++i) {
        PointType po;
        po.x = p[i].local(0); po.y = p[i].local(1); po.z = p[i].local(2);
        po.intensity = p[i].intensity;
        PointType po_w;
        po_w.x = p[i].glob(0); po_w.y = p[i].glob(1); po_w.z = p[i].glob(2);
        switch (p[i].dyn) {
            case CASE1:
                po.normal_x = 1;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE2:
                po.normal_y = p[i].occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            case CASE3:
                po.normal_z = p[i].is_occu_times;
                laserCloudDynObj->push_back(po);
                laserCloudDynObj_world->push_back(po_w);
                break;
            default:
                laserCloudSteadObj->push_back(po_w);
        }
    }

    // Cluster refinement
    if (cluster_coupled) {
        Cluster_.Clusterprocess(dyn_tag_cluster_, *laserCloudDynObj,
                                *laserCloudSteadObj, rot_end, pos_end);
    }

    Points2Buffer(cur_pts, index);
    Buffer2DepthMap(scan_end_time);
    cur_point_soph_pointers_ = (cur_point_soph_pointers_ + 1) % max_pointers_num_;
}


void DynObjFilter::publish_dyn(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dyn,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_std,
    double scan_end_time)
{
    auto stamp = rclcpp::Time(int64_t(scan_end_time * 1e9));

    auto publish = [&](auto & pub, PointCloudXYZI::Ptr & cloud, bool require_subscriber) {
        if (require_subscriber && pub->get_subscription_count() == 0) {
            return;
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp    = stamp;
        msg.header.frame_id = frame_id;
        pub->publish(msg);
    };

    publish(pub_dyn,   laserCloudDynObj, false);
    publish(pub_frame, laserCloudDynObj_world, true);
    publish(pub_std,   laserCloudSteadObj, true);
}
