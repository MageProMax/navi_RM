#include "simple_pgo.h"
#include <gtsam/linear/NoiseModel.h>
#include <algorithm>
#include <cmath>

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(m_config.loop_correspondence_distance);
    m_icp.setCorrespondenceRandomness(20);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << m_config.odom_rotation_sigma, m_config.odom_rotation_sigma, m_config.odom_rotation_sigma, m_config.odom_translation_sigma, m_config.odom_translation_sigma, m_config.odom_translation_sigma).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    if (m_config.planar_graph && idx > 0) {
        // Constrain roll/pitch/z, leave yaw/x/y free for planar graph optimization.
        const auto noise=gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<1e-5,1e-5,1e6,1e6,1e6,1e-5).finished());
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx,gtsam::Pose3(gtsam::Rot3(init_r),gtsam::Point3(init_t)),noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution, int upper_bound)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    if (upper_bound >= 0) max_idx = std::min(max_idx, upper_bound);
    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void SimplePGO::searchForLoopPairs()
{
    m_loop_status = "no_candidate";
    m_last_inlier_ratio = 0.0;
    if (m_key_poses.size() <= static_cast<std::size_t>(m_config.loop_min_index_separation)) return;
    const int current = static_cast<int>(m_key_poses.size())-1;
    const auto &last = m_key_poses.back();
    if (!m_history_pairs.empty() && last.time-m_key_poses[m_history_pairs.back().second].time < m_config.min_loop_detect_duration) {
        m_loop_status = "cooldown"; return;
    }
    std::vector<std::pair<double,int>> candidates;
    double travelled = 0.;
    for (int i=current-1;i>=0;--i) {
        travelled += (m_key_poses[i+1].t_local-m_key_poses[i].t_local).norm();
        if (current-i < m_config.loop_min_index_separation ||
            last.time-m_key_poses[i].time <= m_config.loop_time_tresh || travelled < m_config.loop_min_path_length) continue;
        const double distance=(last.t_global-m_key_poses[i].t_global).norm();
        if (distance < m_config.loop_search_radius) candidates.emplace_back(distance,i);
    }
    std::sort(candidates.begin(),candidates.end());
    auto source=getSubMap(current,0,m_config.submap_resolution);
    if (source->size()<30) { m_loop_status="insufficient_points"; return; }
    for (std::size_t n=0;n<std::min<std::size_t>(3,candidates.size());++n) {
        const int target_id=candidates[n].second;
        // Historical target must not include the current/recent source scans.
        auto target=getSubMap(target_id,m_config.loop_submap_half_range,m_config.submap_resolution,
                              current-m_config.loop_min_index_separation);
        if (target->size()<30) continue;
        CloudType aligned;
        m_icp.setInputSource(source); m_icp.setInputTarget(target);
        m_icp.align(aligned);
        const M4F correction=m_icp.getFinalTransformation();
        const double score=m_icp.getFitnessScore(m_config.loop_correspondence_distance*m_config.loop_correspondence_distance);
        if (!m_icp.hasConverged() || !correction.allFinite() || !std::isfinite(score) || score > m_config.loop_score_tresh) {
            m_loop_status="registration_rejected"; continue;
        }
        const double angle=Eigen::Quaternionf(correction.block<3,3>(0,0)).angularDistance(Eigen::Quaternionf::Identity());
        if (correction.block<3,1>(0,3).norm()>m_config.loop_max_translation || angle>m_config.loop_max_rotation) {
            m_loop_status="correction_rejected"; continue;
        }
        pcl::KdTreeFLANN<PointType> tree;tree.setInputCloud(target);
        std::vector<int> ids(1);std::vector<float> squared(1);std::size_t inliers=0;
        for (const auto &point:aligned)
            if (tree.nearestKSearch(point,1,ids,squared)>0 && squared[0]<=m_config.loop_correspondence_distance*m_config.loop_correspondence_distance) ++inliers;
        m_last_inlier_ratio=static_cast<double>(inliers)/aligned.size();
        if (m_last_inlier_ratio<m_config.loop_min_inlier_ratio) {m_loop_status="overlap_rejected";continue;}
        const M4F delta=m_pending_correction.inverse()*correction;
        if (m_pending_target>=0 && std::abs(target_id-m_pending_target)<=m_config.loop_submap_half_range+1 &&
            delta.block<3,1>(0,3).norm()<0.15f &&
            Eigen::Quaternionf(delta.block<3,3>(0,0)).angularDistance(Eigen::Quaternionf::Identity())<0.087267f)
            ++m_pending_count;
        else m_pending_count=1;
        m_pending_target=target_id; m_pending_correction=correction;
        if (m_pending_count<m_config.loop_confirmations) {m_loop_status="confirming";return;}
        LoopPair pair;pair.source_id=current;pair.target_id=target_id;pair.score=score;
        const M3D r=correction.block<3,3>(0,0).cast<double>()*last.r_global;
        const V3D t=correction.block<3,3>(0,0).cast<double>()*last.t_global+correction.block<3,1>(0,3).cast<double>();
        pair.r_offset=m_key_poses[target_id].r_global.transpose()*r;
        pair.t_offset=m_key_poses[target_id].r_global.transpose()*(t-m_key_poses[target_id].t_global);
        m_cache_pairs.push_back(pair);m_history_pairs.emplace_back(target_id,current);
        m_pending_count=0;m_pending_target=-1;m_loop_status="accepted";return;
    }
    m_pending_count=0;m_pending_target=-1;
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                               gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
                                                               m_config.loop_rotation_sigma,m_config.loop_rotation_sigma,m_config.loop_rotation_sigma,
                                                               m_config.loop_translation_sigma,m_config.loop_translation_sigma,m_config.loop_translation_sigma).finished()))));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}