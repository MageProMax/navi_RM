#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <cmath>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <iomanip>
#include "map_builder/planar_projection.h"

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "base_link";
    std::string body_cloud_frame = "base_link";
    std::string world_frame = "odom";
    bool print_time_cost = false;
    // Pose of the Mid-360 IMU in base_link:
    // p_base = r_bi * p_imu + t_bi.
    M3D r_bi = M3D::Identity();
    V3D t_bi = V3D::Zero();
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        // Receive sensors independently from the relatively expensive mapping
        // callback so that IMU samples are not dropped while a scan is updated.
        m_sensor_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        m_processing_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = m_sensor_cb_group;
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&LIONode::imuCB, this, std::placeholders::_1), sensor_options);
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            m_node_config.lidar_topic, rclcpp::SensorDataQoS().keep_last(20),
            std::bind(&LIONode::lidarCB, this, std::placeholders::_1), sensor_options);

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tracking_pub = this->create_publisher<std_msgs::msg::String>("tracking_status", 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(
            20ms, std::bind(&LIONode::timerCB, this), m_processing_cb_group);
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.body_cloud_frame = config["body_cloud_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        std::vector<double> t_bi_vec = config["t_bi"].as<std::vector<double>>();
        std::vector<double> r_bi_vec = config["r_bi"].as<std::vector<double>>();
        m_node_config.t_bi << t_bi_vec[0], t_bi_vec[1], t_bi_vec[2];
        m_node_config.r_bi << r_bi_vec[0], r_bi_vec[1], r_bi_vec[2],
                              r_bi_vec[3], r_bi_vec[4], r_bi_vec[5],
                              r_bi_vec[6], r_bi_vec[7], r_bi_vec[8];

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.imu_init_duration = config["imu_init_duration"].as<double>();
        m_builder_config.imu_init_max_gyro = config["imu_init_max_gyro"].as<double>();
        m_builder_config.imu_init_max_gyro_std = config["imu_init_max_gyro_std"].as<double>();
        m_builder_config.imu_init_max_acc_std = config["imu_init_max_acc_std"].as<double>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        else if (m_state_data.last_imu_time > 0.0 && timestamp - m_state_data.last_imu_time > 0.02)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Large IMU timestamp gap: %.4f s",
                                 timestamp - m_state_data.last_imu_time);
        }
        // Preserve the sensor's native acceleration unit until IMUProcessor
        // estimates the stationary scale to SI units (Livox: g, Gazebo: SI).
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        else if (m_state_data.last_lidar_time > 0.0 && timestamp - m_state_data.last_lidar_time > 0.3)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Large LiDAR timestamp gap: %.4f s",
                                 timestamp - m_state_data.last_lidar_time);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (!m_state_data.lidar_pushed)
        {
            {
                std::lock_guard<std::mutex> lidar_lock(m_state_data.lidar_mutex);
                if (m_state_data.lidar_buffer.empty())
                    return false;
                m_package.cloud = m_state_data.lidar_buffer.front().second;
                m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
                m_state_data.lidar_pushed = true;

                if (!m_package.cloud || m_package.cloud->empty())
                {
                    m_state_data.lidar_buffer.pop_front();
                    m_state_data.lidar_pushed = false;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Dropping an empty LiDAR scan");
                    return false;
                }
            }
            // Sort outside the buffer lock so sensor reception can continue.
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
        }

        {
            std::scoped_lock sensor_lock(m_state_data.imu_mutex, m_state_data.lidar_mutex);
            if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty() ||
                m_state_data.last_imu_time < m_package.cloud_end_time)
                return false;

            Vec<IMUData>().swap(m_package.imus);
            while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
            {
                m_package.imus.emplace_back(m_state_data.imu_buffer.front());
                m_state_data.imu_buffer.pop_front();
            }
            m_state_data.lidar_buffer.pop_front();
        }
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        M3D r_wb;
        V3D t_wb;
        getPlanarBasePose(r_wb, t_wb);
        odom.pose.pose.position.x = t_wb.x();
        odom.pose.pose.position.y = t_wb.y();
        odom.pose.pose.position.z = t_wb.z();
        Eigen::Quaterniond q(r_wb);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // The filter velocity is the IMU-origin velocity in the world frame.
        // Express only its planar part in the flattened navigation base frame.
        V3D vel = r_wb.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = 0.0;
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        M3D r_wb;
        V3D t_wb;
        getPlanarBasePose(r_wb, t_wb);
        pose.pose.position.x = t_wb.x();
        pose.pose.position.y = t_wb.y();
        pose.pose.position.z = t_wb.z();
        Eigen::Quaterniond q(r_wb);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        M3D r_wb;
        V3D t;
        getPlanarBasePose(r_wb, t);
        Eigen::Quaterniond q(r_wb);
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    /**
     * Convert the filter state (world/odom -> IMU) to world/odom -> base_link.
     * T_w_i = T_w_b * T_b_i, therefore:
     *   R_w_b = R_w_i * R_b_i^T
     *   t_w_b = t_w_i - R_w_b * t_b_i
     */
    void getBasePose(M3D &r_wb, V3D &t_wb) const
    {
        r_wb = m_kf->x().r_wi * m_node_config.r_bi.transpose();
        t_wb = m_kf->x().t_wi - r_wb * m_node_config.t_bi;
    }

    /**
     * Project the full 6-DoF LIO base pose onto the horizontal navigation
     * plane. Nav2 and AMCL require a planar odom -> base_link transform; if
     * roll, pitch, or z leak into it, AMCL compensates them in map -> odom and
     * the local and global costmaps no longer remain coplanar.
     */
    void getPlanarBasePose(M3D &r_wb, V3D &t_wb) const
    {
        M3D r_wb_3d;
        getBasePose(r_wb_3d, t_wb);

        const double yaw = std::atan2(r_wb_3d(1, 0), r_wb_3d(0, 0));
        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);
        r_wb << cos_yaw, -sin_yaw, 0.0,
                 sin_yaw,  cos_yaw, 0.0,
                 0.0,      0.0,     1.0;
        t_wb.z() = 0.0;
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        M3D r_full,r_planar,r_cloud;V3D t_full,t_planar,t_cloud;
        getBasePose(r_full,t_full);getPlanarBasePose(r_planar,t_planar);
        if(!m_have_height_reference){m_initial_base_z=t_full.z();m_have_height_reference=true;}
        planarCloudTransform(r_full,t_full,r_planar,t_planar,m_initial_base_z,r_cloud,t_cloud);

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        // Lidar -> IMU (internal Mid-360 extrinsic), then IMU -> base_link
        // (vehicle installation extrinsic). The published frame now matches
        // the actual point coordinates.
        CloudType::Ptr imu_cloud = m_builder->lidar_processor()->transformCloud(
            m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(
            imu_cloud, m_node_config.r_bi, m_node_config.t_bi);
        body_cloud=m_builder->lidar_processor()->transformCloud(body_cloud,r_cloud,t_cloud);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_cloud_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(body_cloud,r_planar,t_planar);

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        const auto processor=m_builder->lidar_processor();
        std::ostringstream detail;
        detail<<std::setprecision(16)<<"{\"stamp\":"<<m_package.cloud_end_time<<",\"effective_points\":"<<processor->effective_points()
              <<",\"translation_eigen_ratio\":"<<processor->translation_condition_ratio()
              <<",\"weak_translation\":"<<(processor->translation_condition_ratio()<.01?"true":"false")
              <<",\"measurement_updated\":"<<(processor->measurement_updated()?"true":"false")<<"}";
        std_msgs::msg::String tracking;tracking.data=detail.str();m_tracking_pub->publish(tracking);
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_tracking_pub;
    bool m_have_height_reference=false;
    double m_initial_base_z=0.;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::CallbackGroup::SharedPtr m_sensor_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_processing_cb_group;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LIONode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
