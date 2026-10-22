#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "hba/hba.h"
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "interface/srv/refine_map.hpp"
#include "interface/srv/save_poses.hpp"

using namespace std::chrono_literals;
void fromStr(const std::string &str, std::string &file_name, Pose &pose)
{
    std::istringstream ss(str);double qw,qx,qy,qz;std::string extra;
    if(!(ss>>file_name>>pose.t.x()>>pose.t.y()>>pose.t.z()>>qw>>qx>>qy>>qz) || (ss>>extra))
        throw std::invalid_argument("Expected filename tx ty tz qw qx qy qz");
    Eigen::Quaterniond q(qw,qx,qy,qz);
    if(!pose.t.allFinite() || !q.coeffs().allFinite() || q.norm()<1e-8)
        throw std::invalid_argument("Invalid pose or quaternion");
    pose.r=q.normalized().toRotationMatrix();
}

struct NodeConfig
{
    double scan_resolution = 0.1;
};

class HBANode : public rclcpp::Node
{
public:
    HBANode() : Node("hba_node")
    {
        RCLCPP_INFO(this->get_logger(), "HBA node started");
        loadParameters();
        m_hba = std::make_shared<HBA>(m_hba_config);
        m_voxel_grid.setLeafSize(m_node_config.scan_resolution, m_node_config.scan_resolution, m_node_config.scan_resolution);
        m_refine_map_srv = this->create_service<interface::srv::RefineMap>(
            "refine_map",
            std::bind(&HBANode::refineMapCB, this, std::placeholders::_1, std::placeholders::_2));
        m_save_poses_srv = this->create_service<interface::srv::SavePoses>(
            "save_poses",
            std::bind(&HBANode::savePosesCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(100ms, std::bind(&HBANode::mainCB, this));
        m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
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

        m_node_config.scan_resolution = config["scan_resolution"].as<double>();
        m_hba_config.window_size = config["window_size"].as<int>();
        m_hba_config.planar = config["planar"].as<bool>(false);
        m_hba_config.stride = config["stride"].as<int>();
        m_hba_config.voxel_size = config["voxel_size"].as<double>();
        m_hba_config.min_point_num = config["min_point_num"].as<int>();
        m_hba_config.max_layer = config["max_layer"].as<int>();
        m_hba_config.plane_thresh = config["plane_thresh"].as<double>();
        m_hba_config.ba_max_iter = config["ba_max_iter"].as<size_t>();
        m_hba_config.hba_iter = config["hba_iter"].as<size_t>();
        m_hba_config.down_sample = config["down_sample"].as<double>();
        if(m_hba_config.window_size<2 || m_hba_config.stride<2 || m_hba_config.stride>=m_hba_config.window_size ||
           m_hba_config.hba_iter<1 || m_hba_config.ba_max_iter<1 || m_node_config.scan_resolution<=0.)
            throw std::invalid_argument("Invalid HBA configuration");
    }

    void refineMapCB(const std::shared_ptr<interface::srv::RefineMap::Request> request, std::shared_ptr<interface::srv::RefineMap::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        if (!std::filesystem::exists(request->maps_path))
        {
            response->success = false;
            response->message = "maps_path not exists";
            return;
        }

        std::filesystem::path p_dir(request->maps_path);
        std::filesystem::path pcd_dir = p_dir / "patches";
        if (!std::filesystem::exists(pcd_dir))
        {
            response->success = false;
            response->message = pcd_dir.string() + " not exists";
            return;
        }

        std::filesystem::path txt_file = p_dir / "poses.txt";

        if (!std::filesystem::exists(txt_file))
        {
            response->success = false;
            response->message = txt_file.string() + " not exists";
            return;
        }

        try {
        // Load transactionally. A malformed/missing scan must neither crash
        // the process nor silently change the pose-to-patch correspondence.
        auto loaded=std::make_shared<HBA>(m_hba_config);
        std::ifstream ifs(txt_file);
        if(!ifs)throw std::runtime_error("Cannot open poses.txt");
        std::string line;
        std::string file_name;
        Pose pose;
        pcl::PCDReader reader;
        while (std::getline(ifs, line))
        {
            if(line.find_first_not_of(" \t\r\n")==std::string::npos)continue;
            fromStr(line, file_name, pose);
            std::filesystem::path pcd_file = p_dir / "patches" / file_name;
            if (!std::filesystem::exists(pcd_file))
            {
                throw std::runtime_error("Missing patch: "+pcd_file.string());
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            if(reader.read(pcd_file, *cloud)<0 || cloud->empty())
                throw std::runtime_error("Empty or unreadable patch: "+pcd_file.string());
            m_voxel_grid.setInputCloud(cloud);
            m_voxel_grid.filter(*cloud);
            loaded->insert(cloud, pose);
        }
        if(loaded->poses().size()<static_cast<size_t>(m_hba_config.window_size))
            throw std::runtime_error("Need at least window_size poses (default 20)");
        m_hba=std::move(loaded);m_optimized=false;
        RCLCPP_INFO(this->get_logger(), "LOAD POSE %lu;", m_hba->poses().size());
        response->success = true;
        response->message = "load poses success!";
        m_do_optimize = true;
        } catch(const std::exception &error) {
            response->success=false;response->message=error.what();
            RCLCPP_ERROR(get_logger(),"HBA load rejected: %s",error.what());
        }
        return;
    }

    void savePosesCB(const std::shared_ptr<interface::srv::SavePoses::Request> request, std::shared_ptr<interface::srv::SavePoses::Response> response)
    {
        std::lock_guard<std::mutex> lock(m_service_mutex);
        if(!m_optimized || m_do_optimize) {
            response->success=false;response->message="Wait for END OPTIMIZE; no successful optimization to save";return;
        }
        std::filesystem::path file_path(request->file_path);
        std::filesystem::path par_path = file_path.parent_path();
        if (!std::filesystem::exists(par_path))
        {
            response->success = false;
            response->message = "parent path not exists";
            return;
        }
        if (m_hba->poses().size() < 1)
        {
            response->success = false;
            response->message = "poses is empty";
            return;
        }

        try {m_hba->writePoses(file_path);}
        catch(const std::exception &error){response->success=false;response->message=error.what();return;}
        response->success = true;
        response->message = "save poses success!";
        return;
    }
    void mainCB()
    {
        {
            std::lock_guard<std::mutex> lock(m_service_mutex);
            if (!m_do_optimize)
                return;
            RCLCPP_WARN(this->get_logger(), "START OPTIMIZE");
            const auto original=m_hba->poses();
            try {
            publishMap();
            for (size_t i = 0; i < m_hba_config.hba_iter; i++)
            {
                RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu START======", i + 1);
                m_hba->optimize();
                publishMap();
                RCLCPP_INFO(this->get_logger(), "======HBA ITER %lu END========", i + 1);
            }

            m_do_optimize = false;
            m_optimized=true;
            RCLCPP_WARN(this->get_logger(), "END OPTIMIZE");
            } catch(const std::exception &error) {
                m_hba->poses()=original;m_do_optimize=false;m_optimized=false;
                RCLCPP_ERROR(get_logger(),"HBA OPTIMIZE FAILED: %s",error.what());
            }
        }
    }

    void publishMap()
    {
        if (m_cloud_pub->get_subscription_count() < 1)
            return;
        if (m_hba->poses().size() < 1)
            return;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_hba->getMapPoints();
        m_voxel_grid.setInputCloud(cloud);
        m_voxel_grid.filter(*cloud);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
        m_cloud_pub->publish(cloud_msg);
    }

private:
    NodeConfig m_node_config;
    HBAConfig m_hba_config;
    std::shared_ptr<HBA> m_hba;
    rclcpp::Service<interface::srv::RefineMap>::SharedPtr m_refine_map_srv;
    rclcpp::Service<interface::srv::SavePoses>::SharedPtr m_save_poses_srv;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;

    pcl::VoxelGrid<pcl::PointXYZI> m_voxel_grid;
    std::mutex m_service_mutex;
    bool m_do_optimize = false;
    bool m_optimized = false;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HBANode>());
    rclcpp::shutdown();
    return 0;
}
