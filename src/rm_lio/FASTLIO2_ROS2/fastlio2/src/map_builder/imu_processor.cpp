#include "imu_processor.h"

#include <cmath>
#include <iomanip>
#include <iostream>

IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng;
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na;
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg;
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;
    m_last_acc.setZero();
    m_last_gyro.setZero();
    m_imu_cache.clear();
    m_poses_cache.clear();
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());
    if (m_imu_cache.size() < 2)
        return false;

    // Maintain a sliding window that contains at least the requested duration.
    // This makes initialization independent of the actual IMU output rate.
    const double newest_time = m_imu_cache.back().time;
    while (m_imu_cache.size() > 2 &&
           newest_time - m_imu_cache[1].time >= m_config.imu_init_duration)
        m_imu_cache.erase(m_imu_cache.begin());

    const double init_duration = m_imu_cache.back().time - m_imu_cache.front().time;
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num) ||
        init_duration < m_config.imu_init_duration)
        return false;

    V3D acc_mean = V3D::Zero();
    V3D gyro_mean = V3D::Zero();
    for (const auto &imu : m_imu_cache)
    {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(m_imu_cache.size());
    gyro_mean /= static_cast<double>(m_imu_cache.size());

    double acc_variance = 0.0;
    double gyro_variance = 0.0;
    for (const auto &imu : m_imu_cache)
    {
        acc_variance += (imu.acc - acc_mean).squaredNorm();
        gyro_variance += (imu.gyro - gyro_mean).squaredNorm();
    }
    acc_variance /= static_cast<double>(m_imu_cache.size());
    gyro_variance /= static_cast<double>(m_imu_cache.size());
    const double acc_std = std::sqrt(acc_variance);
    const double gyro_std = std::sqrt(gyro_variance);

    // Livox reports acceleration in g while sensor_msgs/Imu (including
    // Gazebo) reports m/s^2. Evaluate stationarity as a fraction of the
    // measured gravity magnitude so the same threshold is unit-independent.
    const double raw_acc_norm = acc_mean.norm();
    if (!std::isfinite(raw_acc_norm) || raw_acc_norm < 1e-6)
    {
        std::cerr << "Invalid mean acceleration during IMU initialization" << std::endl;
        return false;
    }
    const double relative_acc_std = acc_std / raw_acc_norm;

    const bool stationary = gyro_mean.norm() <= m_config.imu_init_max_gyro &&
                            gyro_std <= m_config.imu_init_max_gyro_std &&
                            relative_acc_std <= m_config.imu_init_max_acc_std;
    if (!stationary)
    {
        if (m_last_init_warning_time < 0.0 || newest_time - m_last_init_warning_time >= 2.0)
        {
            std::cerr << "IMU initialization waiting for a stationary 5 s window: "
                      << "|gyro_mean|=" << gyro_mean.norm()
                      << " rad/s, gyro_std=" << gyro_std
                      << " rad/s, relative_acc_std=" << relative_acc_std
                      << " (fraction of g)" << std::endl;
            m_last_init_warning_time = newest_time;
        }
        return false;
    }

    // Livox publishes acceleration in g. Normalize the measured stationary
    // magnitude to SI gravity and apply the same scale to all future samples.
    // For a standards-compliant SI IMU this scale naturally stays near 1.0.
    m_acc_scale = State::gravity / raw_acc_norm;
    for (auto &imu : m_imu_cache)
        imu.acc *= m_acc_scale;
    acc_mean *= m_acc_scale;

    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;
    m_kf->x().bg = gyro_mean;
    if (m_config.gravity_align)
    {
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
    }
    else
        m_kf->x().initGravityDir(-acc_mean);
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;

    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
    std::cout << std::fixed << std::setprecision(6)
              << "IMU initialized with " << m_imu_cache.size()
              << " samples over " << init_duration << " s, native |acc|="
              << raw_acc_norm << ", scale=" << m_acc_scale
              << " SI/native-unit, gyro bias=" << gyro_mean.transpose()
              << std::endl;
    return true;
}

void IMUProcessor::undistort(SyncPackage &package)
{

    for (auto &imu : package.imus)
        imu.acc *= m_acc_scale;

    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // const double imu_time_begin = m_imu_cache.front().time;
    const double imu_time_end = m_imu_cache.back().time;

    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;

    m_poses_cache.clear();
    m_poses_cache.emplace_back(0.0, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);

    V3D acc_val, gyro_val;
    double dt = 0.0;
    Input inp;
    inp.acc = m_imu_cache.back().acc;
    inp.gyro = m_imu_cache.back().gyro;
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); it_imu++)
    {
        IMUData &head = *it_imu;
        IMUData &tail = *(it_imu + 1);
        if (tail.time < m_last_propagate_end_time)
            continue;
        gyro_val = 0.5 * (head.gyro + tail.gyro);
        acc_val = 0.5 * (head.acc + tail.acc);

        if (head.time < m_last_propagate_end_time)
            dt = tail.time - m_last_propagate_end_time;
        else
            dt = tail.time - head.time;

        inp.acc = acc_val;
        inp.gyro = gyro_val;
        m_kf->predict(inp, dt, m_Q);

        m_last_gyro = gyro_val - m_kf->x().bg;
        m_last_acc = m_kf->x().r_wi * (acc_val - m_kf->x().ba) + m_kf->x().g;
        double offset = tail.time - cloud_time_begin;
        m_poses_cache.emplace_back(offset, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);
    }

    dt = propagate_time_end - imu_time_end;
    m_kf->predict(inp, dt, m_Q);
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    M3D cur_r_wi = m_kf->x().r_wi;
    V3D cur_t_wi = m_kf->x().t_wi;
    M3D cur_r_il = m_kf->x().r_il;
    V3D cur_t_il = m_kf->x().t_il;
    auto it_pcl = package.cloud->points.end() - 1;

    for (auto it_kp = m_poses_cache.end() - 1; it_kp != m_poses_cache.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->trans;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == package.cloud->points.begin())
                break;
        }
    }
}
