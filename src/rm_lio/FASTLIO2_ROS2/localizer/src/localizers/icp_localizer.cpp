#include "icp_localizer.h"
#include <cmath>
#include <stdexcept>
#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    if (config.registration_method != "gicp" && config.registration_method != "icp")
        throw std::invalid_argument("registration_method must be gicp or icp");
    if (config.covariance_neighbors < 3 ||
        !std::isfinite(config.rough_correspondence_distance) || config.rough_correspondence_distance <= 0 ||
        !std::isfinite(config.refine_correspondence_distance) || config.refine_correspondence_distance <= 0 ||
        !std::isfinite(config.min_inlier_ratio) || config.min_inlier_ratio <= 0 || config.min_inlier_ratio > 1)
        throw std::invalid_argument("Invalid registration correspondence/overlap settings");
    m_rough_gicp.setCorrespondenceRandomness(config.covariance_neighbors);
    m_refine_gicp.setCorrespondenceRandomness(config.covariance_neighbors);
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    if (reader.read(path, *cloud) < 0 || cloud->empty())
        return false;
    auto finite = std::make_shared<CloudType>();
    for (const auto &point : *cloud) if (pcl::isFinite(point)) finite->push_back(point);
    if (finite->empty()) return false;
    cloud = finite;
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
    // Set targets only when the map changes, allowing GICP to cache covariances.
    m_rough_gicp.setInputTarget(m_rough_tgt);
    m_refine_gicp.setInputTarget(m_refine_tgt);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_refine_icp.setInputTarget(m_refine_tgt);
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    m_inlier_ratio = 0.0;
    m_fitness = -1.0;
    const std::size_t minimum = m_config.registration_method == "gicp" ?
        static_cast<std::size_t>(m_config.covariance_neighbors) : 3;
    if (!guess.allFinite() || m_refine_inp->size() < minimum || m_rough_inp->size() < minimum ||
        m_refine_tgt->size() < minimum || m_rough_tgt->size() < minimum) return false;
    auto stage = [](auto &registration, const CloudType::Ptr &source,
                    int iterations, double distance, double threshold, M4F &pose) {
        CloudType aligned;
        registration.setMaximumIterations(iterations);
        registration.setMaxCorrespondenceDistance(distance);
        registration.setTransformationEpsilon(1e-6);
        registration.setInputSource(source);
        registration.align(aligned, pose);
        // PCL getFitnessScore range is squared Euclidean distance.
        const double score = registration.getFitnessScore(distance * distance);
        if (!registration.hasConverged() || !std::isfinite(score) || score > threshold ||
            !registration.getFinalTransformation().allFinite()) return false;
        pose = registration.getFinalTransformation();
        return true;
    };
    M4F candidate = guess;
    bool ok;
    if (m_config.registration_method == "gicp") {
        ok = stage(m_rough_gicp, m_rough_inp, m_config.rough_max_iteration,
                   m_config.rough_correspondence_distance, m_config.rough_score_thresh, candidate) &&
             stage(m_refine_gicp, m_refine_inp, m_config.refine_max_iteration,
                   m_config.refine_correspondence_distance, m_config.refine_score_thresh, candidate);
    } else {
        ok = stage(m_rough_icp, m_rough_inp, m_config.rough_max_iteration,
                   m_config.rough_correspondence_distance, m_config.rough_score_thresh, candidate) &&
             stage(m_refine_icp, m_refine_inp, m_config.refine_max_iteration,
                   m_config.refine_correspondence_distance, m_config.refine_score_thresh, candidate);
    }
    if (!ok) return false;
    // A low residual over a tiny matched subset is not a valid localization.
    CloudType aligned;
    pcl::transformPointCloud(*m_refine_inp, aligned, candidate);
    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(m_refine_tgt);
    std::vector<int> indices(1); std::vector<float> distances(1);
    std::size_t inliers = 0; double squared_error = 0;
    const double limit = m_config.refine_correspondence_distance * m_config.refine_correspondence_distance;
    for (const auto &point : aligned) {
        if (pcl::isFinite(point) && tree.nearestKSearch(point, 1, indices, distances) > 0 && distances[0] <= limit) {
            ++inliers; squared_error += distances[0];
        }
    }
    m_inlier_ratio = static_cast<double>(inliers) / aligned.size();
    if (inliers) m_fitness = squared_error / inliers;
    if (m_inlier_ratio < m_config.min_inlier_ratio) return false;
    guess = candidate;
    return true;
}
