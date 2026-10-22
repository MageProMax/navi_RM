#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <interface/srv/save_maps.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <atomic>
#include <filesystem>
#include <condition_variable>
#include <optional>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>
#include "pgos/occupancy_builder.h"
#include "pgos/map_export.h"
using namespace std::chrono_literals;

class OnlinePGO : public rclcpp::Node {
    using Sync=message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,nav_msgs::msg::Odometry>>;
    struct Snapshot {std::vector<KeyPoseWithCloud> frames;std::size_t revision;M3D r;V3D t;};
public:
    OnlinePGO():Node("online_pgo") {
        Config c;c.planar_graph=true;
        c.key_pose_delta_trans=declare_parameter("key_pose_delta_trans",.20);
        c.key_pose_delta_deg=declare_parameter("key_pose_delta_deg",10.);
        c.loop_search_radius=declare_parameter("loop_search_radius",2.5);
        c.loop_time_tresh=declare_parameter("loop_time_threshold",15.);
        c.loop_score_tresh=declare_parameter("loop_score_threshold",.02);
        c.min_loop_detect_duration=declare_parameter("loop_interval",10.);
        c.loop_submap_half_range=declare_parameter("loop_submap_half_range",5);
        c.submap_resolution=declare_parameter("submap_resolution",.10);
        c.loop_min_index_separation=declare_parameter("loop_min_index_separation",15);
        c.loop_min_path_length=declare_parameter("loop_min_path_length",4.);
        c.loop_confirmations=declare_parameter("loop_confirmations",2);
        c.loop_correspondence_distance=declare_parameter("loop_correspondence_distance",.5);
        c.loop_min_inlier_ratio=declare_parameter("loop_min_inlier_ratio",.65);
        c.loop_max_translation=declare_parameter("loop_max_translation",1.0);
        c.loop_max_rotation=declare_parameter("loop_max_rotation",.261799);
        maximum_frames_=declare_parameter("maximum_keyframes",3000);
        grid_config_.resolution=declare_parameter("resolution",.05);
        grid_config_.minimum_z=declare_parameter("min_obstacle_height",.1);
        grid_config_.maximum_z=declare_parameter("max_obstacle_height",1.5);
        grid_config_.ground_z=declare_parameter("min_ground_height",-.15);
        filter_config_.enabled=declare_parameter("static_filter.enabled",true);
        filter_config_.voxel_size=declare_parameter("static_filter.voxel_size",.10);
        filter_config_.endpoint_margin=declare_parameter("static_filter.endpoint_margin",.20);
        filter_config_.min_observation_interval=declare_parameter("static_filter.min_observation_interval",.5);
        filter_config_.min_free_observations=declare_parameter("static_filter.min_free_observations",3);
        filter_config_.free_to_hit_ratio=declare_parameter("static_filter.free_to_hit_ratio",3.);
        filter_config_.max_range=declare_parameter("static_filter.max_range",10.);
        const int voxels=declare_parameter("static_filter.maximum_voxels",2000000);
        const int steps=declare_parameter("static_filter.maximum_ray_steps",60000000);
        if(voxels<100||steps<100)throw std::invalid_argument("Invalid filter resource bounds");
        filter_config_.maximum_voxels=voxels;filter_config_.maximum_ray_steps=steps;
        validateStaticFilter(filter_config_);
        stationary_interval_=declare_parameter("static_filter.stationary_observation_interval",1.);
        if(!std::isfinite(stationary_interval_)||stationary_interval_<.5)throw std::invalid_argument("Invalid stationary observation interval");
        const double period=declare_parameter("map_update_interval",1.);
        if(c.key_pose_delta_trans<=0||c.key_pose_delta_deg<=0||c.loop_search_radius<=0||c.loop_time_tresh<=0||
           c.loop_score_tresh<=0||c.loop_confirmations<2||c.loop_min_index_separation<5||c.submap_resolution<=0||
           c.loop_correspondence_distance<=0||c.loop_min_inlier_ratio<=0||c.loop_min_inlier_ratio>1||
           c.loop_max_translation<=0||c.loop_max_rotation<=0||maximum_frames_<20||grid_config_.resolution<=0||
           grid_config_.ground_z>=grid_config_.minimum_z||grid_config_.minimum_z>=grid_config_.maximum_z||period<=0)
            throw std::invalid_argument("Invalid online PGO configuration");
        pgo_=std::make_shared<SimplePGO>(c);
        const auto retained=rclcpp::QoS(1).reliable().transient_local();
        map_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/map",retained);
        ready_pub_=create_publisher<std_msgs::msg::Bool>("/pgo/map_ready",rclcpp::QoS(10).reliable().transient_local());
        status_pub_=create_publisher<std_msgs::msg::String>("/pgo/status",retained);
        filter_pub_=create_publisher<std_msgs::msg::String>("/pgo/static_filter_status",retained);
        path_pub_=create_publisher<nav_msgs::msg::Path>("/pgo/optimized_path",retained);
        tf_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        auto qos=rclcpp::QoS(10).get_rmw_qos_profile();
        cloud_sub_.subscribe(this,"/fastlio2/body_cloud",qos);
        odom_sub_.subscribe(this,"/fastlio2/lio_odom",qos);
        sync_=std::make_shared<Sync>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,nav_msgs::msg::Odometry>(10),cloud_sub_,odom_sub_);
        sync_->registerCallback(std::bind(&OnlinePGO::receive,this,std::placeholders::_1,std::placeholders::_2));
        process_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        process_timer_=create_wall_timer(100ms,std::bind(&OnlinePGO::process,this),process_group_);
        tf_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        tf_timer_=create_wall_timer(20ms,std::bind(&OnlinePGO::broadcast,this),tf_group_);
        refresh_timer_=create_wall_timer(1s,[this]() {
            std::lock_guard<std::mutex> lock(map_mutex_);
            // A stationary robot still has a valid static map; keep its receipt
            // fresh without adding redundant graph nodes or re-optimizing it.
            if(last_map_ && !fault_ && last_map_revision_==revision_.load()) map_pub_->publish(*last_map_);
        });
        map_period_=period;
        save_srv_=create_service<interface::srv::SaveMaps>("/pgo/save_maps",std::bind(&OnlinePGO::save,this,std::placeholders::_1,std::placeholders::_2));
        setReady(false);worker_=std::thread(&OnlinePGO::mapWorker,this);
        RCLCPP_INFO(get_logger(),"Online 3D GICP/PGO mapping ready; sole map->odom and /map authority");
    }
    ~OnlinePGO() override {
        {std::lock_guard<std::mutex> lock(map_mutex_);stop_=true;}condition_.notify_all();
        if(worker_.joinable())worker_.join();
    }
private:
    void failMapping() {std::lock_guard<std::mutex> lock(map_mutex_);fault_=true;setReady(false);}
    void setReady(bool value) {std_msgs::msg::Bool m;m.data=value;ready_pub_->publish(m);}
    void status(const std::string &state,std::size_t count,std::size_t loops,const std::string &loop_status,double inlier) {
        std_msgs::msg::String m;std::ostringstream s;
        s<<"{\"state\":\""<<state<<"\",\"revision\":"<<revision_.load()<<",\"keyframes\":"<<count
         <<",\"loops\":"<<loops<<",\"loop_status\":\""<<loop_status<<"\",\"inlier_ratio\":"<<inlier<<"}";
        m.data=s.str();status_pub_->publish(m);
    }
    void receive(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
        const double stamp=rclcpp::Time(cloud->header.stamp).seconds();
        if(std::abs(stamp-rclcpp::Time(odom->header.stamp).seconds())>.02 || cloud->header.frame_id!="base_link"||
           odom->child_frame_id!="base_link"||odom->header.frame_id!="odom")return;
        const auto &q=odom->pose.pose.orientation;const auto &p=odom->pose.pose.position;
        Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);V3D position(p.x,p.y,p.z);
        if(!rotation.coeffs().allFinite()||rotation.norm()<1e-6||!position.allFinite())return;
        CloudType::Ptr raw(new CloudType),finite(new CloudType),filtered(new CloudType);
        pcl::fromROSMsg(*cloud,*raw);for(const auto &point:*raw)if(pcl::isFinite(point))finite->push_back(point);
        if(finite->size()<30)return;
        pcl::VoxelGrid<PointType> voxel;voxel.setLeafSize(.08,.08,.08);voxel.setInputCloud(finite);voxel.filter(*filtered);
        CloudWithPose cp;cp.cloud=filtered;cp.pose.r=rotation.normalized().toRotationMatrix();cp.pose.t=position;
        cp.pose.setTime(cloud->header.stamp.sec,cloud->header.stamp.nanosec);
        std::lock_guard<std::mutex> lock(input_mutex_);
        if(have_stamp_&&stamp<=rclcpp::Time(latest_stamp_).seconds())return;
        latest_stamp_=cloud->header.stamp;have_stamp_=true;pending_=cp;
    }
    void process() {
        std::optional<CloudWithPose> input;
        {std::lock_guard<std::mutex> lock(input_mutex_);input.swap(pending_);}
        if(!input)return;
        if((now()-rclcpp::Time(input->pose.sec,input->pose.nsec,RCL_ROS_TIME)).seconds()>.5)return;
        std::lock_guard<std::mutex> graph_lock(graph_mutex_);
        if(pgo_->keyPoses().size()>=static_cast<std::size_t>(maximum_frames_)) {
            failMapping();status("keyframe_limit",pgo_->keyPoses().size(),pgo_->historyPairs().size(),"stopped",0);return;
        }
        const bool key=pgo_->addKeyPose(*input);
        if(key) {
            pgo_->searchForLoopPairs();const bool loop=pgo_->hasLoop();
            if(loop){std::lock_guard<std::mutex> commit_lock(map_mutex_);++revision_;setReady(false);RCLCPP_INFO(get_logger(),"Accepted 3D loop; rebuilding all historical occupancy, revision=%zu",revision_.load());}
            pgo_->smoothAndUpdate();dirty_=true;last_observation_time_=input->pose.second;
        } else if(filter_config_.enabled && input->pose.second-last_observation_time_>=stationary_interval_) {
            // Evidence while parked must not create odometry graph nodes. Keep
            // up to six recent scans per anchor, at most 3000 extra scans total.
            const auto anchor=pgo_->keyPoses().size()-1;
            auto &observations=stationary_[anchor];
            if(observations.size()>=6){observations.erase(observations.begin());--stationary_count_;}
            if(stationary_count_<3000){observations.push_back(*input);++stationary_count_;dirty_=true;}
            last_observation_time_=input->pose.second;
        }
        // Coalesce snapshots; a dedicated worker rebuilds the grid, never blocks TF.
        const double seconds=now().seconds();
        if(dirty_ && (seconds-last_map_request_>=map_period_||revision_.load()!=last_requested_revision_)) {
            Snapshot snapshot{mappingObservations(),revision_.load(),pgo_->offsetR(),pgo_->offsetT()};
            {std::lock_guard<std::mutex> lock(map_mutex_);next_map_=std::move(snapshot);}
            condition_.notify_one();dirty_=false;last_map_request_=seconds;last_requested_revision_=revision_.load();
        }
        status("tracking",pgo_->keyPoses().size(),pgo_->historyPairs().size(),pgo_->loopStatus(),pgo_->lastInlierRatio());
    }
    std::vector<KeyPoseWithCloud> mappingObservations() {
        auto frames=pgo_->keyPoses();
        for(const auto &group:stationary_) {
            const auto &anchor=pgo_->keyPoses().at(group.first);
            for(const auto &scan:group.second) {
                KeyPoseWithCloud f;
                const M3D relative=anchor.r_local.transpose()*scan.pose.r;
                const V3D offset=anchor.r_local.transpose()*(scan.pose.t-anchor.t_local);
                f.r_global=anchor.r_global*relative;f.t_global=anchor.r_global*offset+anchor.t_global;
                f.r_local=scan.pose.r;f.t_local=scan.pose.t;f.time=scan.pose.second;f.body_cloud=scan.cloud;
                frames.push_back(f);
            }
        }
        std::stable_sort(frames.begin(),frames.end(),[](const auto &a,const auto &b){return a.time<b.time;});
        return frames;
    }
    void broadcast() {
        builtin_interfaces::msg::Time stamp;
        {std::lock_guard<std::mutex> lock(input_mutex_);if(!have_stamp_)return;stamp=latest_stamp_;}
        const double clock=now().seconds();
        if(clock+.5<last_broadcast_clock_) {
            failMapping();RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,"Clock reset; restart online mapping");return;
        }
        last_broadcast_clock_=clock;
        const double age=clock-rclcpp::Time(stamp).seconds();
        // /clock and sensor messages use independent DDS paths; a temporarily
        // future scan is not evidence that the simulation clock was reset.
        if(age < -.05 || age>.5)return;
        M3D r;V3D t;{std::lock_guard<std::mutex> lock(offset_mutex_);r=active_r_;t=active_t_;}
        // Planar navigation frame, matching the planar LIO input. Grid rebuilds
        // use the same planar optimized poses (see mapWorker).
        const double yaw=std::atan2(r(1,0),r(0,0));
        geometry_msgs::msg::TransformStamped msg;msg.header.frame_id="map";msg.child_frame_id="odom";msg.header.stamp=stamp;
        msg.transform.translation.x=t.x();msg.transform.translation.y=t.y();
        msg.transform.rotation.z=std::sin(yaw/2);msg.transform.rotation.w=std::cos(yaw/2);tf_->sendTransform(msg);
    }
    void mapWorker() {
        while(true) {
            Snapshot snapshot;
            {std::unique_lock<std::mutex> lock(map_mutex_);condition_.wait(lock,[&]{return stop_||next_map_.has_value();});
             if(stop_) return;
             snapshot=std::move(*next_map_);next_map_.reset();}
            try {
                // Apply identical optimized planar poses to filtering, PCD and raster.
                // Raw observations are retained separately for later reprocessing.
                for(auto &f:snapshot.frames) {
                    const double yaw=std::atan2(f.r_global(1,0),f.r_global(0,0));
                    f.r_global=Eigen::AngleAxisd(yaw,V3D::UnitZ()).toRotationMatrix();f.t_global.z()=0.;
                }
                auto filtered=filterStaticMap(snapshot.frames,filter_config_);
                auto grid=buildOccupancy(filtered.frames,grid_config_);
                if(filtered.stats.state.find("budget")!=std::string::npos)
                    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),10000,"Static-map filter: %s (all points retained)",filtered.stats.state.c_str());
                if(!rclcpp::ok()) return;
                std::lock_guard<std::mutex> commit_lock(map_mutex_);
                if(fault_ || snapshot.revision!=revision_.load())continue;
                grid.header.stamp=now();grid.info.map_load_time=grid.header.stamp;
                {std::lock_guard<std::mutex> lock(offset_mutex_);active_r_=snapshot.r;active_t_=snapshot.t;}
                last_map_=std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);last_map_revision_=snapshot.revision;
                last_raw_frames_=snapshot.frames;last_filtered_=std::move(filtered);
                std_msgs::msg::String filter_status;
                const auto &fs=last_filtered_.stats;std::ostringstream detail;
                detail<<"{\"state\":\""<<fs.state<<"\",\"input_points\":"<<fs.input_points
                      <<",\"removed_points\":"<<fs.removed_points<<",\"removed_voxels\":"<<fs.removed_voxels
                      <<",\"ray_steps\":"<<fs.ray_steps<<",\"revision\":"<<snapshot.revision<<"}";
                filter_status.data=detail.str();filter_pub_->publish(filter_status);
                map_pub_->publish(grid);
                nav_msgs::msg::Path path;path.header=grid.header;
                for(const auto &f:snapshot.frames) {
                    geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
                    pose.pose.position.x=f.t_global.x();pose.pose.position.y=f.t_global.y();
                    Eigen::Quaterniond q(f.r_global);pose.pose.orientation.w=q.w();pose.pose.orientation.x=q.x();pose.pose.orientation.y=q.y();pose.pose.orientation.z=q.z();path.poses.push_back(pose);
                }
                path_pub_->publish(path);setReady(true);
            } catch(const std::exception &e) {
                if(!rclcpp::ok()) return;
                failMapping();RCLCPP_ERROR(get_logger(),"Map rebuild failed: %s",e.what());
            }
        }
    }
    void save(const std::shared_ptr<interface::srv::SaveMaps::Request> req,std::shared_ptr<interface::srv::SaveMaps::Response> res) {
        std::vector<KeyPoseWithCloud> raw;
        StaticFilterResult filtered;nav_msgs::msg::OccupancyGrid grid;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if(fault_ || !last_map_ || last_map_revision_!=revision_.load()) {
                res->message="Wait for a committed map /pgo/map_ready before saving";return;
            }
            raw=last_raw_frames_;filtered=last_filtered_;grid=*last_map_;
        }
        if(filtered.frames.empty()||!std::filesystem::is_directory(req->file_path)) {
            res->message="Directory must exist and mapping must contain keyframes";return;
        }
        try {
            const std::filesystem::path dir(req->file_path);
            // One committed snapshot supplies BOTH maps, including after loops.
            // Patches stay RAW, so thresholds and optimized poses can be revisited.
            if(req->save_patches)writePatches(dir,raw);
            writePairedMap(dir,filtered,grid,filter_config_);
            res->success=true;res->message="Saved paired static maps; filter="+filtered.stats.state+
                ", removed_points="+std::to_string(filtered.stats.removed_points)+"; patches are raw";
        }catch(const std::exception &e){res->message=e.what();}
    }

    StaticFilterConfig filter_config_;StaticFilterResult last_filtered_;std::vector<KeyPoseWithCloud> last_raw_frames_;
    std::map<std::size_t,std::vector<CloudWithPose>> stationary_;std::size_t stationary_count_=0;
    double stationary_interval_=1.,last_observation_time_=-1e9;
    std::shared_ptr<SimplePGO> pgo_;GridConfig grid_config_;int maximum_frames_;double map_period_,last_map_request_=-1e9,last_broadcast_clock_=0.;
    bool dirty_=false,have_stamp_=false,stop_=false;std::atomic<bool> fault_{false};std::atomic<std::size_t> revision_{0};std::size_t last_requested_revision_=0;
    std::mutex graph_mutex_,input_mutex_,map_mutex_,offset_mutex_;std::condition_variable condition_;std::thread worker_;
    std::optional<CloudWithPose> pending_;std::optional<Snapshot> next_map_;builtin_interfaces::msg::Time latest_stamp_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> last_map_;std::size_t last_map_revision_=0;
    M3D active_r_=M3D::Identity();V3D active_t_=V3D::Zero();
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;std::shared_ptr<Sync> sync_;
    rclcpp::CallbackGroup::SharedPtr process_group_,tf_group_;rclcpp::TimerBase::SharedPtr process_timer_,tf_timer_,refresh_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,filter_pub_;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr save_srv_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);auto node=std::make_shared<OnlinePGO>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);executor.add_node(node);executor.spin();node.reset();rclcpp::shutdown();}
