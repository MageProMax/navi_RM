#include "pgos/static_map_filter.h"
#include "pgos/occupancy_builder.h"
#include "pgos/map_export.h"
#include <iostream>
#include <stdexcept>
#include <unistd.h>

void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
const V3D person(2.03,.03,.53), pole(2.03,.83,.53), sensor(0,.22157,.33018);
const V3D wall=sensor+(person-sensor)*2.;
KeyPoseWithCloud frame(double time,const std::vector<V3D> &points,V3D position=V3D::Zero(),double yaw=0) {
    KeyPoseWithCloud f;f.r_global=Eigen::AngleAxisd(yaw,V3D::UnitZ()).toRotationMatrix();f.t_global=position;
    f.r_local=f.r_global;f.t_local=f.t_global;f.time=time;f.body_cloud=std::make_shared<CloudType>();
    for(const auto &v:points){const V3D local=f.r_global.transpose()*(v-position);PointType p;p.x=local.x();p.y=local.y();p.z=local.z();p.intensity=7;f.body_cloud->push_back(p);}return f;
}
bool contains(const std::vector<KeyPoseWithCloud> &frames,const V3D &v) {
    for(const auto &f:frames)for(const auto &p:*f.body_cloud)
        if((f.r_global*p.getVector3fMap().cast<double>()+f.t_global-v).norm()<1e-4)return true;
    return false;
}
int value(const nav_msgs::msg::OccupancyGrid &g,const V3D &v){
    int x=std::floor((v.x()-g.info.origin.position.x)/g.info.resolution),y=std::floor((v.y()-g.info.origin.position.y)/g.info.resolution);
    return x<0||y<0||x>=static_cast<int>(g.info.width)||y>=static_cast<int>(g.info.height)?-1:g.data[y*g.info.width+x];
}
int main() {
    StaticFilterConfig c;
    std::vector<KeyPoseWithCloud> frames{frame(0,{person,pole,V3D(1,-.4,0)})};
    for(int i=1;i<=5;++i)frames.push_back(frame(i,{wall,V3D(1,-.4,0),V3D(person.x(),person.y(),0)}));
    const auto raw_count=frames.front().body_cloud->size();
    auto cleaned=filterStaticMap(frames,c);
    require(cleaned.stats.state=="filtered" && cleaned.stats.removed_points==1,"moving person not removed");
    require(!contains(cleaned.frames,person),"person still in static PCD observations");
    require(contains(cleaned.frames,pole),"single-view thin static obstacle deleted without free evidence");
    require(contains(cleaned.frames,wall),"background wall deleted");
    require(frames.front().body_cloud->size()==raw_count && contains(frames,person),"raw/live observations modified");
    auto grid=buildOccupancy(cleaned.frames,GridConfig{});
    require(value(grid,person)==0,"person ghost survived in 2-D map");
    require(value(grid,pole)==100,"thin obstacle missing from 2-D map");
    require(value(grid,wall)==100,"wall missing from 2-D map");
    require(value(grid,V3D(.1,-.3,0))==-1,"unobserved space cleared");
    // Rays over a person's head must not clear the body voxel via XY projection.
    auto high=frames;for(std::size_t i=1;i<high.size();++i)high[i]=frame(i,{V3D(wall.x(),wall.y(),2.)});
    require(contains(filterStaticMap(high,c).frames,person),"high ray cleared a different height");
    // No free-space evidence behind an occluding measured return.
    auto occluded=frames;for(std::size_t i=1;i<occluded.size();++i)occluded[i]=frame(i,{sensor+(person-sensor)*.5});
    require(contains(filterStaticMap(occluded,c).frames,person),"occluded voxel falsely cleared");
    // Many points or many scans within one time window count only once.
    std::vector<KeyPoseWithCloud> burst{frame(0,{person})};
    for(int i=0;i<20;++i)burst.push_back(frame(1.+.001*i,std::vector<V3D>(50,wall)));
    require(contains(filterStaticMap(burst,c).frames,person),"one high-density observation manufactured confirmations");
    // Correct transforms, including a rotated body frame and an optimized map shift.
    std::vector<KeyPoseWithCloud> rotated;
    for(std::size_t i=0;i<frames.size();++i){std::vector<V3D> points;for(const auto &p:*frames[i].body_cloud)points.push_back(p.getVector3fMap().cast<double>());rotated.push_back(frame(i,points,V3D(-.1,.05,0),.2));}
    require(!contains(filterStaticMap(rotated,c).frames,person),"body/optimized map transforms inconsistent");
    for(auto &f:frames)f.t_global+=V3D(4,-2,0);
    const auto shifted=filterStaticMap(frames,c);
    require(!contains(shifted.frames,person+V3D(4,-2,0)),"loop rebuild retained stale dynamic classification");
    // Reappearing, persistent structure can gain occupancy again after rebuilding.
    for(int i=6;i<16;++i)frames.push_back(frame(i,{person+V3D(4,-2,0)},V3D(4,-2,0)));
    require(contains(filterStaticMap(frames,c).frames,person+V3D(4,-2,0)),"static support cannot recover");
    c.maximum_ray_steps=100;const auto budget=filterStaticMap(frames,c);
    require(budget.stats.state=="ray_budget_retained_all" && budget.stats.removed_points==0,"partial budget result erased obstacles");
    c.enabled=false;require(filterStaticMap(frames,c).stats.removed_points==0,"disabled filter deleted points");
    c=StaticFilterConfig{};
    const auto output=std::filesystem::temp_directory_path()/("static_filter_test_"+std::to_string(getpid()));
    std::filesystem::create_directories(output);
    writePairedMap(output,cleaned,grid,c);
    CloudType saved;require(pcl::io::loadPCDFile((output/"map.pcd").string(),saved)==0,"paired PCD missing");
    for(const auto &p:saved)require((p.getVector3fMap().cast<double>()-person).norm()>.01,"saved PCD contains removed person");
    const auto metadata=YAML::LoadFile((output/"filter_metadata.yaml").string());require(metadata["removed_points"].as<int>()==1,"export metadata mismatch");
    require(std::filesystem::exists(output/"map.yaml") && std::filesystem::exists(output/"map.pgm"),"paired grid missing");
    std::filesystem::remove_all(output);
    std::cout<<"PASS moving person, single-view pole, walls, unknown, height, occlusion, independent observations, optimized poses, raw immutability, budgets and paired export\n";
}
