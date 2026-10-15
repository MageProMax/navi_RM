#include <queue>
#include <chrono>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <pcl/common/point_tests.h>
#include <mutex>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "localizers/tracking_guard.h"
#include "localizers/scan_history.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "odom";
    double update_hz = 5.0;
    double accumulation_seconds = 0.5;
    int accumulation_max_frames = 10;
    double accumulation_voxel = 0.08;
    int accumulation_max_points_per_frame = 30000;
    double sync_tolerance = 0.02;
    double max_scan_age = 0.5;
    double max_tracking_translation = 0.30;
    double max_tracking_rotation = 0.1745329252; // 10 degrees, full 3D rotation
    bool planar_tf = true;
    bool auto_relocalize = false;
    std::string map_path;
    std::vector<double> initial_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    builtin_interfaces::msg::Time last_message_time;
    ScanHistory history;
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        if (m_config.auto_relocalize)
        {
            if (!setRelocalizationTarget(m_config.map_path, m_config.initial_pose))
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Automatic 3D relocalization setup failed; map->odom will not be published");
            }
        }

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_metrics_pub = this->create_publisher<std_msgs::msg::String>("registration_status", 10);
        RCLCPP_INFO(get_logger(), "Registration backend=%s, accumulation=%.2fs/%d frames",
                    m_localizer_config.registration_method.c_str(), m_config.accumulation_seconds,
                    m_config.accumulation_max_frames);
        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        // 创建一个互斥的回调组专属给 Timer 使用
        m_timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // 创建 Timer 时，将其绑定到这个新的回调组
        m_timer = this->create_wall_timer(
            10ms, 
            std::bind(&LocalizerNode::timerCB, this), 
            m_timer_cb_group // 传入回调组
        );
        // ICP may take longer than the navigation TF tolerance. Publish the
        // last accepted offset independently using the newest sensor stamp.
        m_tf_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_tf_timer = this->create_wall_timer(10ms, [this]() {
            builtin_interfaces::msg::Time stamp;
            {
                std::lock_guard<std::mutex> lock(m_state.message_mutex);
                if (!m_state.message_received) return;
                stamp = m_state.last_message_time;
            }
            sendBroadCastTF(stamp);
        }, m_tf_cb_group);
    
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        // Keep the original standalone YAML interface for backwards
        // compatibility.  When config_path is empty, standard ROS parameters
        // (including the combined Nav2/3D-localizer parameter file) are used.
        if (!config_path.empty())
        {
            YAML::Node config = YAML::LoadFile(config_path);
            if (!config)
            {
                RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
                if (config["registration_method"]) m_localizer_config.registration_method = config["registration_method"].as<std::string>();
                if (config["covariance_neighbors"]) m_localizer_config.covariance_neighbors = config["covariance_neighbors"].as<int>();
                if (config["rough_correspondence_distance"]) m_localizer_config.rough_correspondence_distance = config["rough_correspondence_distance"].as<double>();
                if (config["refine_correspondence_distance"]) m_localizer_config.refine_correspondence_distance = config["refine_correspondence_distance"].as<double>();
                if (config["min_inlier_ratio"]) m_localizer_config.min_inlier_ratio = config["min_inlier_ratio"].as<double>();
                if (config["accumulation_seconds"]) m_config.accumulation_seconds = config["accumulation_seconds"].as<double>();
                if (config["accumulation_max_frames"]) m_config.accumulation_max_frames = config["accumulation_max_frames"].as<int>();
                if (config["accumulation_voxel"]) m_config.accumulation_voxel = config["accumulation_voxel"].as<double>();
                if (config["accumulation_max_points_per_frame"]) m_config.accumulation_max_points_per_frame = config["accumulation_max_points_per_frame"].as<int>();
                if (config["sync_tolerance"]) m_config.sync_tolerance = config["sync_tolerance"].as<double>();
                if (config["max_scan_age"]) m_config.max_scan_age = config["max_scan_age"].as<double>();
                m_config.cloud_topic = config["cloud_topic"].as<std::string>();
                m_config.odom_topic = config["odom_topic"].as<std::string>();
                m_config.map_frame = config["map_frame"].as<std::string>();
                m_config.local_frame = config["local_frame"].as<std::string>();
                m_config.update_hz = config["update_hz"].as<double>();
                if (config["max_tracking_translation"])
                    m_config.max_tracking_translation = config["max_tracking_translation"].as<double>();
                if (config["max_tracking_rotation"])
                    m_config.max_tracking_rotation = config["max_tracking_rotation"].as<double>();

                m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
                m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
                m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
                m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

                m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
                m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
                m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
                m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
            }
        }

        m_localizer_config.registration_method = declare_parameter<std::string>("registration_method", m_localizer_config.registration_method);
        m_localizer_config.covariance_neighbors = declare_parameter<int>("covariance_neighbors", m_localizer_config.covariance_neighbors);
        m_localizer_config.rough_correspondence_distance = declare_parameter<double>("rough_correspondence_distance", m_localizer_config.rough_correspondence_distance);
        m_localizer_config.refine_correspondence_distance = declare_parameter<double>("refine_correspondence_distance", m_localizer_config.refine_correspondence_distance);
        m_localizer_config.min_inlier_ratio = declare_parameter<double>("min_inlier_ratio", m_localizer_config.min_inlier_ratio);
        m_config.accumulation_seconds = declare_parameter<double>("accumulation_seconds", m_config.accumulation_seconds);
        m_config.accumulation_max_frames = declare_parameter<int>("accumulation_max_frames", m_config.accumulation_max_frames);
        m_config.accumulation_voxel = declare_parameter<double>("accumulation_voxel", m_config.accumulation_voxel);
        m_config.accumulation_max_points_per_frame = declare_parameter<int>("accumulation_max_points_per_frame", m_config.accumulation_max_points_per_frame);
        m_config.sync_tolerance = declare_parameter<double>("sync_tolerance", m_config.sync_tolerance);
        m_config.max_scan_age = declare_parameter<double>("max_scan_age", m_config.max_scan_age);
        if (!std::isfinite(m_config.accumulation_seconds) || m_config.accumulation_seconds < 0 ||
            m_config.accumulation_seconds > 2.0 || m_config.accumulation_max_frames < 1 ||
            m_config.accumulation_max_frames > 40 || !std::isfinite(m_config.accumulation_voxel) ||
            m_config.accumulation_voxel <= 0 || m_config.accumulation_max_points_per_frame < 100 ||
            !std::isfinite(m_config.sync_tolerance) || m_config.sync_tolerance <= 0 ||
            !std::isfinite(m_config.max_scan_age) || m_config.max_scan_age <= 0)
            throw std::invalid_argument("Invalid accumulation/synchronization settings");
        m_config.cloud_topic = this->declare_parameter<std::string>("cloud_topic", m_config.cloud_topic);
        m_config.odom_topic = this->declare_parameter<std::string>("odom_topic", m_config.odom_topic);
        m_config.map_frame = this->declare_parameter<std::string>("map_frame", m_config.map_frame);
        m_config.local_frame = this->declare_parameter<std::string>("local_frame", m_config.local_frame);
        m_config.update_hz = this->declare_parameter<double>("update_hz", m_config.update_hz);
        m_config.planar_tf = this->declare_parameter<bool>("planar_tf", m_config.planar_tf);
        m_config.auto_relocalize = this->declare_parameter<bool>("auto_relocalize", false);
        m_config.map_path = this->declare_parameter<std::string>("map_path", "");
        m_config.initial_pose = this->declare_parameter<std::vector<double>>(
            "initial_pose", m_config.initial_pose);
        if (m_config.initial_pose.size() != 6)
            throw std::invalid_argument(
                "initial_pose must contain [x, y, z, roll, pitch, yaw] in metres/radians");
        // Scalar overrides let launch files keep Gazebo's spawn pose and the
        // ICP initial guess synchronized without constructing a parameter array
        // from launch substitutions.
        m_config.initial_pose[0] = this->declare_parameter<double>(
            "initial_x", m_config.initial_pose[0]);
        m_config.initial_pose[1] = this->declare_parameter<double>(
            "initial_y", m_config.initial_pose[1]);
        m_config.initial_pose[2] = this->declare_parameter<double>(
            "initial_z", m_config.initial_pose[2]);
        m_config.initial_pose[3] = this->declare_parameter<double>(
            "initial_roll", m_config.initial_pose[3]);
        m_config.initial_pose[4] = this->declare_parameter<double>(
            "initial_pitch", m_config.initial_pose[4]);
        m_config.initial_pose[5] = this->declare_parameter<double>(
            "initial_yaw", m_config.initial_pose[5]);

        m_localizer_config.rough_scan_resolution = this->declare_parameter<double>(
            "rough_scan_resolution", m_localizer_config.rough_scan_resolution);
        m_localizer_config.rough_map_resolution = this->declare_parameter<double>(
            "rough_map_resolution", m_localizer_config.rough_map_resolution);
        m_localizer_config.rough_max_iteration = this->declare_parameter<int>(
            "rough_max_iteration", m_localizer_config.rough_max_iteration);
        m_localizer_config.rough_score_thresh = this->declare_parameter<double>(
            "rough_score_thresh", m_localizer_config.rough_score_thresh);
        m_localizer_config.refine_scan_resolution = this->declare_parameter<double>(
            "refine_scan_resolution", m_localizer_config.refine_scan_resolution);
        m_localizer_config.refine_map_resolution = this->declare_parameter<double>(
            "refine_map_resolution", m_localizer_config.refine_map_resolution);
        m_localizer_config.refine_max_iteration = this->declare_parameter<int>(
            "refine_max_iteration", m_localizer_config.refine_max_iteration);
        m_localizer_config.refine_score_thresh = this->declare_parameter<double>(
            "refine_score_thresh", m_localizer_config.refine_score_thresh);

        m_config.max_tracking_translation = this->declare_parameter<double>(
            "max_tracking_translation", m_config.max_tracking_translation);
        m_config.max_tracking_rotation = this->declare_parameter<double>(
            "max_tracking_rotation", m_config.max_tracking_rotation);
        if (!std::isfinite(m_config.max_tracking_translation) ||
            m_config.max_tracking_translation <= 0.0 ||
            !std::isfinite(m_config.max_tracking_rotation) ||
            m_config.max_tracking_rotation <= 0.0)
            throw std::invalid_argument("Tracking correction limits must be finite and positive");

        if (!std::isfinite(m_config.update_hz) || m_config.update_hz <= 0.0)
            throw std::invalid_argument("update_hz must be greater than zero");
    }
    void timerCB()
    {
        // Serialize map replacement with alignment AND acceptance of its result.
        std::lock_guard<std::mutex> operation_lock(m_operation_mutex);
        M3D current_local_r;
        V3D current_local_t;
        ScanHistory history;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (!m_state.message_received)
                return;
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            history = m_state.history;
        }
        const auto now = this->get_clock()->now();
        const double scan_age = (now - rclcpp::Time(current_time)).seconds();
        if (scan_age < -0.05 || scan_age > m_config.max_scan_age) return;
        if (now < m_state.last_send_tf_time)
            m_state.last_send_tf_time = now;
        const auto diff = now - m_state.last_send_tf_time;
        if (diff.seconds() <= 1.0 / m_config.update_hz ||
            (m_have_processed_scan && current_time == m_processed_stamp))
        {
            return;
        }
        m_state.last_send_tf_time = now;
        m_processed_stamp = current_time;
        m_have_processed_scan = true;

        M4F initial_guess = M4F::Identity();
        bool relocalization_pending = false;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            relocalization_pending = m_state.service_received;
            if (relocalization_pending)
                initial_guess = m_state.initial_guess;
            else if (!m_state.localize_success)
                return;
        }
        if (!relocalization_pending)
        {
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * current_local_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * current_local_t + m_state.last_offset_t).cast<float>();
        }
        const M4F prediction = initial_guess;

        const auto registration_start = std::chrono::steady_clock::now();
        auto current_cloud = history.latestBodyCloud();
        bool result = false;
        {
            std::lock_guard<std::mutex> lock(m_localizer_mutex);
            m_localizer->setInput(current_cloud);
            result = m_localizer->align(initial_guess);
        }
        if (result)
        {
            const auto correction = checkTrackingCorrection(
                prediction, initial_guess, m_config.max_tracking_translation,
                m_config.max_tracking_rotation, relocalization_pending);
            if (!correction.accepted)
            {
                result = false;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Rejected ICP correction: translation=%.3f m rotation=%.2f deg; "
                    "limits=%.3f m/%.2f deg. Keeping previous map->odom",
                    correction.translation, correction.rotation * 180.0 / M_PI,
                    m_config.max_tracking_translation,
                    m_config.max_tracking_rotation * 180.0 / M_PI);
            }
        }
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (relocalization_pending)
            {
                m_state.localize_success = true;
                m_state.service_received = false;
                RCLCPP_INFO(this->get_logger(),
                            "Initial 3D relocalization succeeded; publishing map -> odom");
            }
        }
        else if (relocalization_pending)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "3D relocalization has not converged; check initial_pose and map pairing");
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "3D tracking update rejected or ICP failed; retaining map->odom "
                "and following LIO odometry. Persistent failures require relocalization");
        }
        std_msgs::msg::String metrics;
        std::ostringstream json;
        json << "{\"backend\":\"" << m_localizer_config.registration_method
             << "\",\"accepted\":" << (result ? "true" : "false")
             << ",\"frames\":" << history.scans.size()
             << ",\"points\":" << current_cloud->size()
             << ",\"inlier_ratio\":" << m_localizer->inlierRatio()
             << ",\"fitness_m2\":" << m_localizer->fitness()
             << ",\"duration_ms\":" << std::chrono::duration<double,std::milli>(
                    std::chrono::steady_clock::now()-registration_start).count() << "}";
        metrics.data = json.str(); m_metrics_pub->publish(metrics);
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        const double stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
        if (std::abs(stamp-rclcpp::Time(odom_msg->header.stamp).seconds()) > m_config.sync_tolerance ||
            cloud_msg->header.frame_id != odom_msg->child_frame_id ||
            odom_msg->header.frame_id != m_config.local_frame) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                "Dropping cloud/odom pair: timestamp or frame mismatch");
            return;
        }
        const auto &q = odom_msg->pose.pose.orientation;
        Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        const auto &p = odom_msg->pose.pose.position;
        V3D translation(p.x,p.y,p.z);
        if (!rotation.coeffs().allFinite() || rotation.norm()<1e-6 || !translation.allFinite()) return;
        rotation.normalize();
        auto raw = std::make_shared<CloudType>();
        pcl::fromROSMsg(*cloud_msg, *raw);
        auto finite = std::make_shared<CloudType>();
        for (const auto &point : *raw) if (pcl::isFinite(point)) finite->push_back(point);
        if (finite->empty()) return;
        auto cloud = std::make_shared<CloudType>();
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(m_config.accumulation_voxel,m_config.accumulation_voxel,m_config.accumulation_voxel);
        voxel.setInputCloud(finite); voxel.filter(*cloud);
        if (cloud->size() > static_cast<std::size_t>(m_config.accumulation_max_points_per_frame)) {
            auto limited = std::make_shared<CloudType>();
            for (int i=0;i<m_config.accumulation_max_points_per_frame;++i)
                limited->push_back((*cloud)[static_cast<std::size_t>(i)*cloud->size()/m_config.accumulation_max_points_per_frame]);
            cloud = limited;
        }
        M4F pose = M4F::Identity();
        pose.block<3,3>(0,0)=rotation.toRotationMatrix().cast<float>();
        pose.block<3,1>(0,3)=translation.cast<float>();
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        m_state.history.push({stamp,pose,cloud},m_config.accumulation_seconds,m_config.accumulation_max_frames);
        m_state.last_r = rotation.toRotationMatrix();
        m_state.last_t = translation;
        m_state.last_message_time = cloud_msg->header.stamp;
        m_state.message_received = true;
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        M3D rotation;
        V3D t;
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            if (!m_state.localize_success) return;
            rotation = m_state.last_offset_r;
            t = m_state.last_offset_t;
        }
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(rotation);
        if (m_config.planar_tf)
        {
            const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
            q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            t.z() = 0.0;
        }
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        const std::vector<double> initial_pose = {
            request->x, request->y, request->z,
            request->roll, request->pitch, request->yaw};
        response->success = setRelocalizationTarget(request->pcd_path, initial_pose);
        response->message = response->success ? "relocalize request accepted" :
                                                "failed to load map or initial pose";
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud;
        {
            std::lock_guard<std::mutex> lock(m_localizer_mutex);
            map_cloud = m_localizer->refineMap();
        }
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

    bool setRelocalizationTarget(
        const std::string & pcd_path, const std::vector<double> & initial_pose)
    {
        std::lock_guard<std::mutex> operation_lock(m_operation_mutex);
        if (initial_pose.size() != 6)
        {
            RCLCPP_ERROR(this->get_logger(), "Initial pose must contain 6 values");
            return false;
        }
        for (const auto value : initial_pose)
            if (!std::isfinite(value))
                return false;
        if (pcd_path.empty() || !std::filesystem::exists(pcd_path))
        {
            RCLCPP_ERROR(this->get_logger(), "PCD map not found: %s", pcd_path.c_str());
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(m_localizer_mutex);
            if (!m_localizer->loadMap(pcd_path))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load PCD map: %s", pcd_path.c_str());
                return false;
            }
        }

        const Eigen::AngleAxisd roll_angle(
            initial_pose[3], Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch_angle(
            initial_pose[4], Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw_angle(
            initial_pose[5], Eigen::Vector3d::UnitZ());
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) =
                (yaw_angle * pitch_angle * roll_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(
                initial_pose[0], initial_pose[1], initial_pose[2]);
            m_have_processed_scan = false;
            m_state.service_received = true;
            m_state.localize_success = false;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            m_state.history.clear();
            m_state.message_received = false;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded 3D map: %s", pcd_path.c_str());
        return true;
    }

private:
    rclcpp::CallbackGroup::SharedPtr m_timer_cb_group; // 新增这一行
    rclcpp::CallbackGroup::SharedPtr m_tf_cb_group;
    rclcpp::TimerBase::SharedPtr m_tf_timer;
    NodeConfig m_config;

    NodeState m_state;
    std::mutex m_localizer_mutex;
    std::mutex m_operation_mutex;
    bool m_have_processed_scan = false;
    builtin_interfaces::msg::Time m_processed_stamp;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_metrics_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LocalizerNode>();
    
    // 使用多线程执行器，默认线程数等于 CPU 核心数
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Starting Localizer Node with MultiThreadedExecutor");
    executor.spin(); // 开始多线程并发调度
    
    rclcpp::shutdown();
    return 0;
}
