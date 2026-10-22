#include "hba.h"
#include "graph_information.h"
#include <gtsam/slam/PriorFactor.h>
#include <iomanip>
#include <filesystem>
HBA::HBA(const HBAConfig config) : m_config(config)
{
    m_poses.clear();
    m_clouds.clear();
    m_lbas.clear();
}
void HBA::insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    m_clouds.push_back(cloud);
    m_poses.push_back(pose);
}
void HBA::optimize()
{
    m_levels = calcLevels();
    gtsam::Values initial_estimates;
    gtsam::LevenbergMarquardtParams lm_params;
    gtsam::NonlinearFactorGraph graph;

    // for (size_t i = 0; i < m_config.hba_iter; i++)
    // {
    Vec<Vec<BLAM>>().swap(m_lbas);
    m_lbas.resize(m_levels, Vec<BLAM>());
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds = m_clouds;
    Vec<Pose> poses = m_poses;
    initial_estimates.clear();
    graph.resize(0);
    // 设置初始值

    std::cout << "INIT POSE ESTIMATE" << std::endl;
    for (size_t j = 0; j < m_poses.size(); j++)
    {
        initial_estimates.insert(j, gtsam::Pose3(gtsam::Rot3(poses[j].r), gtsam::Point3(poses[j].t)));
        if(m_config.planar && j>0) {
            const auto noise=gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector6()<<1e-8,1e-8,1e12,1e12,1e12,1e-8).finished());
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(j,initial_estimates.at<gtsam::Pose3>(j),noise));
        }
    }
    // BA has a six-dimensional gauge freedom. Keep the original map frame
    // instead of allowing every offline run to tilt/translate the whole map.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,initial_estimates.at<gtsam::Pose3>(0),
        gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones()*1e-8)));

    for (int level = 0; level < m_levels; level++)
    {
        constructHierarchy(clouds, poses, level);
        std::cout << "LBA OPTIMIZE LEVEL: " << level << std::endl;

        updateCloudsAndPose(clouds, poses, level);
    }
    Vec<std::pair<size_t, size_t>> between_factors_id;
    Vec<Pose> between_factors_pose;
    Vec<M6D> between_factors_info;
    getAllFactors(between_factors_id, between_factors_pose, between_factors_info);
    // 添加二元因子

    std::cout << "CONSTRUCT BETWEEN FACTORS" << std::endl;
    for (size_t j = 0; j < between_factors_id.size(); j++)
    {
        gtsam::BetweenFactor<gtsam::Pose3> between_factor(between_factors_id[j].first,
                                                          between_factors_id[j].second,
                                                          gtsam::Pose3(gtsam::Rot3(between_factors_pose[j].r), gtsam::Point3(between_factors_pose[j].t)),
                                                          gtsam::noiseModel::Gaussian::Information(relativeInformation(between_factors_info[j])));
        graph.add(between_factor);
    }
    // 优化
    if(between_factors_id.empty())throw std::runtime_error("No HBA relative constraints");
    const double before_error=graph.error(initial_estimates);
    if(!std::isfinite(before_error))throw std::runtime_error("Invalid HBA graph objective");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimates, lm_params);

    std::cout << "LM OPTIMIZE " << std::endl;
    gtsam::Values result = optimizer.optimize();
    const double after_error=graph.error(result);
    if(!std::isfinite(after_error) || after_error>before_error+1e-6*std::max(1.,before_error))
        throw std::runtime_error("HBA graph optimization did not produce a finite improving result");
    std::cout<<"HBA GRAPH ERROR "<<before_error<<" -> "<<after_error<<std::endl;
    // 更新位姿
    std::cout << "UPDATE POSE ESTIMATE" << std::endl;
    for (size_t j = 0; j < m_poses.size(); j++)
    {
        gtsam::Pose3 pose = result.at<gtsam::Pose3>(j);
        if(!pose.matrix().allFinite())throw std::runtime_error("Non-finite optimized HBA pose");
        m_poses[j].t = pose.translation().matrix().cast<double>();
        m_poses[j].r = pose.rotation().matrix().cast<double>();
    }
    // }
}
void HBA::updateCloudsAndPose(Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, Vec<Pose> &poses, int level)
{
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(clouds);
    Vec<Pose>().swap(poses);
    for (BLAM &lba : m_lbas[level])
    {
        poses.push_back(lba.poses()[0]);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = lba.getLocalCloud();
        if (m_config.down_sample > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
            voxel_grid.setLeafSize(m_config.down_sample, m_config.down_sample, m_config.down_sample);
            voxel_grid.setInputCloud(cloud);
            voxel_grid.filter(*cloud);
        }
        clouds.push_back(cloud);
    }
}
void HBA::constructHierarchy(Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, Vec<Pose> &poses, int level)
{
    BLAMConfig config;
    config.voxel_size = m_config.voxel_size;
    config.max_iter = m_config.ba_max_iter;
    config.max_layer = m_config.max_layer;
    config.min_point_num = m_config.min_point_num;
    config.plane_thresh = m_config.plane_thresh;
    config.planar = m_config.planar;
    for (size_t i = 0; i < clouds.size(); i += m_config.stride)
    {
        m_lbas[level].emplace_back(config);
        bool drop_last = false;
        for (int j = 0; j < m_config.window_size; j++)
        {
            size_t idx = i + j;
            if (i + j >= clouds.size())
            {
                drop_last = true;
                break;
            }
            m_lbas[level].back().insert(clouds[idx], poses[idx]);
        }
        // std::cout << "level: " << level << " window: " << i << std::endl;
        m_lbas[level].back().optimize();
        if (drop_last)
            break;
    }
}
void HBA::getAllFactors(Vec<std::pair<size_t, size_t>> &ids, Vec<Pose> &poses, Vec<M6D> &infos)
{
    for (size_t level_idx = 0; level_idx < static_cast<size_t>(m_levels); level_idx++)
    {
        for (size_t stride_idx = 0; stride_idx < m_lbas[level_idx].size(); stride_idx++)
        {
            BLAM &blam = m_lbas[level_idx][stride_idx];
            for (size_t pose_idx = 0; pose_idx < blam.poses().size() - 1; pose_idx++)
            {
                size_t from = pose_idx + stride_idx * m_config.stride;
                size_t to = pose_idx + 1 + stride_idx * m_config.stride;
                from = from * std::pow(m_config.stride, level_idx);
                to = to * std::pow(m_config.stride, level_idx);
                ids.emplace_back(std::make_pair(from, to));
                Pose &pose_from = blam.poses()[pose_idx];
                Pose &pose_to = blam.poses()[pose_idx + 1];
                Pose pose_fc;
                pose_fc.r = pose_from.r.transpose() * pose_to.r;
                pose_fc.t = pose_from.r.transpose() * (pose_to.t - pose_from.t);
                // std::cout << "level: " << level_idx << " from: " << from << " to: " << to << std::endl;
                poses.push_back(pose_fc);
                infos.push_back(blam.H().block<6, 6>(6 * pose_idx, 6 * (pose_idx + 1)));
            }
        }
    }
}
int HBA::calcLevels()
{

    assert(m_poses.size() > 0 && m_config.window_size > 0 && m_config.stride > 0);
    return static_cast<int>(0.5 * std::log(3.0 * std::pow(static_cast<double>(m_poses.size()), 2) * (std::pow(m_config.stride, 3) - m_config.stride) / std::pow(static_cast<double>(m_config.window_size), 3)) / std::log(m_config.stride));
}
pcl::PointCloud<pcl::PointXYZI>::Ptr HBA::getMapPoints()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        Pose &pose = m_poses[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_clouds[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud, *transformed, pose.t, Eigen::Quaterniond(pose.r));
        *ret += *transformed;
    }
    return ret;
}
void HBA::writePoses(const std::string &path)
{
    const auto temporary=path+".tmp";
    std::ofstream txt_file;txt_file.exceptions(std::ios::failbit|std::ios::badbit);
    txt_file.open(temporary);txt_file<<std::setprecision(17);
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        Pose &pose = m_poses[i];
        V3D t = pose.t;
        Eigen::Quaterniond q(pose.r);
        if(!t.allFinite()||!q.coeffs().allFinite()||q.norm()<1e-8)throw std::runtime_error("Invalid optimized pose");
        q.normalize();
        txt_file << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    }
    txt_file.close();
    std::filesystem::rename(temporary,path);
}
