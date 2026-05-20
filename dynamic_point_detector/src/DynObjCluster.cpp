// Copyright 2025 Pan — Apache-2.0
// DynObjCluster: ROS2 adaptation of M-Detector voxel clustering + EA_disk prediction

#include "cluster_predict/voxel_cluster.h"
#include <queue>
#include <limits>

void DynObjCluster::Init() {}

void DynObjCluster::buildVoxelMap(
    const pcl::PointCloud<PointType> & cloud,
    std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> & vmap)
{
    float inv = 1.f / Voxel_resolution;
    for (int i = 0; i < int(cloud.size()); ++i) {
        const auto & pt = cloud.points[i];
        VoxelKey k{int(std::floor(pt.x * inv)),
                   int(std::floor(pt.y * inv)),
                   int(std::floor(pt.z * inv))};
        vmap[k].push_back(i);
    }
}

void DynObjCluster::clusterBFS(
    const std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> & vmap,
    std::vector<std::vector<int>> & clusters)
{
    std::unordered_set<size_t> visited;
    VoxelHash hasher;

    for (auto & [seed_key, seed_pts] : vmap) {
        if (visited.count(hasher(seed_key))) continue;

        std::vector<int> cluster;
        std::queue<VoxelKey> q;
        q.push(seed_key);
        visited.insert(hasher(seed_key));

        while (!q.empty()) {
            VoxelKey cur = q.front(); q.pop();
            auto it = vmap.find(cur);
            if (it != vmap.end())
                for (int idx : it->second) cluster.push_back(idx);

            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (!dx && !dy && !dz) continue;
                        VoxelKey nb{cur.x+dx, cur.y+dy, cur.z+dz};
                        if (!visited.count(hasher(nb)) && vmap.count(nb)) {
                            visited.insert(hasher(nb));
                            q.push(nb);
                        }
                    }
        }

        if (int(cluster.size()) >= min_cluster_size
            && int(cluster.size()) <= max_cluster_size)
            clusters.push_back(std::move(cluster));
    }
}

void DynObjCluster::Clusterprocess(
    std::vector<int> & dyn_tag,
    const pcl::PointCloud<PointType> & event_point,
    const pcl::PointCloud<PointType> & /*raw_point*/,
    const Eigen::Matrix3d & /*rot*/,
    const Eigen::Vector3d & /*pos*/)
{
    predictions.clear();
    if (event_point.empty()) return;

    std::unordered_map<VoxelKey, std::vector<int>, VoxelHash> vmap;
    buildVoxelMap(event_point, vmap);

    std::vector<std::vector<int>> clusters;
    clusterBFS(vmap, clusters);

    std::fill(dyn_tag.begin(), dyn_tag.end(), 0);

    // Compute cluster centres + build EA_disk predictions
    std::vector<Eigen::Vector2d> cur_centres;
    cur_centres.reserve(clusters.size());

    for (auto & cl : clusters) {
        double sx = 0, sy = 0, max_r = 0;
        for (int idx : cl) {
            if (idx >= int(event_point.size())) continue;
            dyn_tag[idx] = 1;
            sx += event_point.points[idx].x;
            sy += event_point.points[idx].y;
        }
        double cx = sx / cl.size(), cy = sy / cl.size();
        for (int idx : cl) {
            if (idx >= int(event_point.size())) continue;
            double r = std::hypot(event_point.points[idx].x - cx,
                                  event_point.points[idx].y - cy);
            max_r = std::max(max_r, r);
        }
        cur_centres.emplace_back(cx, cy);

        EA_disk d;
        d.cx = cx; d.cy = cy;
        d.a  = std::max(max_r + 0.1, 0.3);
        d.b  = d.a;
        // velocity: match to nearest previous centre
        if (!prev_centres_.empty()) {
            double best = std::numeric_limits<double>::max();
            int    best_i = -1;
            for (int i = 0; i < int(prev_centres_.size()); ++i) {
                double dist = (prev_centres_[i] - Eigen::Vector2d(cx,cy)).norm();
                if (dist < best) { best = dist; best_i = i; }
            }
            if (best_i >= 0 && best < 2.0 && prev_time_ > 0) {
                double dt = 0.1; // nominal frame dt
                d.vx = (cx - prev_centres_[best_i].x()) / dt;
                d.vy = (cy - prev_centres_[best_i].y()) / dt;
            }
        }
        predictions.push_back(d);
    }

    prev_centres_ = cur_centres;
    prev_time_ = 1.0;  // mark as initialized (actual dt passed per-call)
}
