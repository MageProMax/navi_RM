#pragma once
#include "static_map_filter.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>

inline YAML::Node filterConfigYaml(const StaticFilterConfig &c) {
    YAML::Node n;n["enabled"]=c.enabled;n["voxel_size"]=c.voxel_size;n["endpoint_margin"]=c.endpoint_margin;
    n["min_observation_interval"]=c.min_observation_interval;n["min_free_observations"]=c.min_free_observations;
    n["free_to_hit_ratio"]=c.free_to_hit_ratio;n["max_range"]=c.max_range;
    n["maximum_voxels"]=c.maximum_voxels;n["maximum_ray_steps"]=c.maximum_ray_steps;return n;
}
inline StaticFilterConfig readFilterConfig(const std::filesystem::path &path) {
    StaticFilterConfig c;auto n=YAML::LoadFile(path.string());
    if(n["online_pgo"])n=YAML::Clone(n["online_pgo"]["ros__parameters"]["static_filter"]);
    if(n["enabled"])c.enabled=n["enabled"].as<bool>();
    if(n["voxel_size"])c.voxel_size=n["voxel_size"].as<double>();
    if(n["endpoint_margin"])c.endpoint_margin=n["endpoint_margin"].as<double>();
    if(n["min_observation_interval"])c.min_observation_interval=n["min_observation_interval"].as<double>();
    if(n["min_free_observations"])c.min_free_observations=n["min_free_observations"].as<int>();
    if(n["free_to_hit_ratio"])c.free_to_hit_ratio=n["free_to_hit_ratio"].as<double>();
    if(n["max_range"])c.max_range=n["max_range"].as<double>();
    if(n["maximum_voxels"])c.maximum_voxels=n["maximum_voxels"].as<std::size_t>();
    if(n["maximum_ray_steps"])c.maximum_ray_steps=n["maximum_ray_steps"].as<std::size_t>();
    validateStaticFilter(c);return c;
}
inline void writeYaml(const std::filesystem::path &file,const YAML::Node &node) {
    std::ofstream out(file);out<<node<<'\n';if(!out)throw std::runtime_error("Cannot write "+file.string());
}
inline void writeCloud(const std::filesystem::path &file,const CloudType &cloud) {
    if(!cloud.empty()) {
        if(pcl::io::savePCDFileBinary(file.string(),cloud)<0)throw std::runtime_error("Cannot write "+file.string());
    }else {
        std::ofstream out(file);out<<"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA ascii\n";
        if(!out)throw std::runtime_error("Cannot write empty cloud");
    }
}
inline void writePatches(const std::filesystem::path &dir,const std::vector<KeyPoseWithCloud> &frames) {
    std::filesystem::create_directories(dir/"patches");std::ofstream poses(dir/"poses.txt"),times(dir/"frame_times.txt");
    poses<<std::setprecision(17);times<<std::setprecision(17);
    for(std::size_t i=0;i<frames.size();++i) {
        const auto &f=frames[i];const auto name=std::to_string(i)+".pcd";
        writeCloud(dir/"patches"/name,*f.body_cloud);Eigen::Quaterniond q(f.r_global);
        poses<<name<<' '<<f.t_global.transpose()<<' '<<q.w()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<'\n';times<<f.time<<'\n';
    }
    if(!poses||!times)throw std::runtime_error("Cannot write poses/timestamps");
}
inline void writeFilterMetadata(const std::filesystem::path &dir,const StaticFilterConfig &c,const StaticFilterStats &s) {
    writeYaml(dir/"dynamic_filter.yaml",filterConfigYaml(c));
    YAML::Node n;n["state"]=s.state;n["input_points"]=s.input_points;n["removed_points"]=s.removed_points;
    n["removed_voxels"]=s.removed_voxels;n["candidate_voxels"]=s.candidate_voxels;n["ray_steps"]=s.ray_steps;
    writeYaml(dir/"filter_metadata.yaml",n);
}
inline void writePairedMap(const std::filesystem::path &dir,const StaticFilterResult &filtered,
                           const nav_msgs::msg::OccupancyGrid &grid,const StaticFilterConfig &cfg) {
    CloudType merged;
    for(const auto &f:filtered.frames){CloudType world;pcl::transformPointCloud(*f.body_cloud,world,f.t_global,Eigen::Quaterniond(f.r_global));merged+=world;}
    if(merged.empty())throw std::runtime_error("No points in static map");
    writeCloud(dir/"map.pcd",merged);
    std::ofstream image(dir/"map.pgm",std::ios::binary);image<<"P5\n"<<grid.info.width<<' '<<grid.info.height<<"\n255\n";
    for(int y=static_cast<int>(grid.info.height)-1;y>=0;--y)for(unsigned x=0;x<grid.info.width;++x) {
        const auto v=grid.data[y*grid.info.width+x];const unsigned char pixel=v<0?205:(v>=65?0:254);
        image.write(reinterpret_cast<const char*>(&pixel),1);
    }
    if(!image)throw std::runtime_error("Cannot write map.pgm");
    YAML::Node map;map["image"]="map.pgm";map["resolution"]=grid.info.resolution;
    map["origin"].push_back(grid.info.origin.position.x);map["origin"].push_back(grid.info.origin.position.y);map["origin"].push_back(0.);
    map["negate"]=0;map["occupied_thresh"]=.65;map["free_thresh"]=.196;writeYaml(dir/"map.yaml",map);
    writeFilterMetadata(dir,cfg,filtered.stats);
    const auto &first=filtered.frames.front();std::ofstream pose(dir/"start_pose.env");pose<<std::setprecision(12)
        <<"export NAV_INITIAL_X="<<first.t_global.x()<<"\nexport NAV_INITIAL_Y="<<first.t_global.y()
        <<"\nexport NAV_INITIAL_YAW="<<std::atan2(first.r_global(1,0),first.r_global(0,0))<<'\n';
    if(!pose)throw std::runtime_error("Cannot write start_pose.env");
}
