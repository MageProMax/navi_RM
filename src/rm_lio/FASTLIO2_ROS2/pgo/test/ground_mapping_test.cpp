#include "pgos/occupancy_builder.h"
#include <iostream>

void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
CloudType floorCloud(double a=.035,double b=-.02,double offset=.01) {
    CloudType cloud;
    for(int i=-20;i<=20;++i)for(int j=-20;j<=20;++j) {
        PointType p;p.x=i*.15+.013;p.y=j*.15+.017;
        p.z=a*p.x+b*p.y+offset+.003*std::sin(i*7+j*3);p.intensity=1.;cloud.push_back(p);
    }
    return cloud;
}
int value(const nav_msgs::msg::OccupancyGrid &g,double x,double y) {
    const int col=std::floor((x-g.info.origin.position.x)/g.info.resolution);
    const int row=std::floor((y-g.info.origin.position.y)/g.info.resolution);
    return col<0||row<0||col>=static_cast<int>(g.info.width)||row>=static_cast<int>(g.info.height)?-1:g.data[row*g.info.width+col];
}
int main() {
    GroundPlaneConfig cfg;GroundPlaneTracker tracker(cfg);
    const auto floor=floorCloud();auto plane=tracker.update(floor,M3D::Identity(),V3D::Zero(),0.);
    require(plane.has_value(),"supported sloping floor not fitted");
    require((*plane-V3D(.035,-.02,.01)).norm()<.003,"incorrect floor fit");
    auto dense_wall=floor;
    for(int i=-40;i<=40;++i)for(int j=0;j<40;++j) {
        PointType p;p.x=1.5;p.y=i*.07;p.z=.01*j;dense_wall.push_back(p);
    }
    plane=tracker.update(dense_wall,M3D::Identity(),V3D::Zero(),.1);
    require(plane&&(*plane-V3D(.035,-.02,.01)).norm()<.006,"wall contaminated floor fit");
    // Track slow height drift across gaps larger than the unsupported-reuse age.
    for(int i=1;i<=15;++i) {
        const double z=.01+i*.02;
        plane=tracker.update(floorCloud(.035,-.02,z),M3D::Identity(),V3D::Zero(),i*1.1);
        require(plane&&std::abs(plane->z()-z)<.004,"slow ground-height drift lost");
    }
    GroundPlaneTracker no_floor(cfg);
    require(!no_floor.update(floorCloud(0,0,.12),M3D::Identity(),V3D::Zero(),0.),"raised surface established arbitrary initial floor");
    require(!no_floor.update(floorCloud(.4,0,0),M3D::Identity(),V3D::Zero(),.1),"steep plane accepted");
    CloudType line;
    for(int i=0;i<100;++i){PointType p;p.x=i*.03;p.y=0;p.z=.02*p.x;line.push_back(p);}
    require(!no_floor.update(line,M3D::Identity(),V3D::Zero(),.2),"line established ground plane");
    // Supported prior must transform with vehicle translation and rotation.
    GroundPlaneTracker reuse(cfg);
    require(reuse.update(floor,M3D::Identity(),V3D::Zero(),0.).has_value(),"reuse setup failed");
    const M3D rotation=Eigen::AngleAxisd(std::acos(-1.)/2.,V3D::UnitZ()).toRotationMatrix();
    const V3D position(.2,.1,0);
    const auto transformed=reuse.update(CloudType{},rotation,position,.5);
    require(transformed&&(*transformed-V3D(-.02,-.035,.015)).norm()<.004,"cached world plane transformed incorrectly");
    require(!reuse.update(CloudType{},rotation,position,2.),"expired unsupported model reused");
    require(!reuse.update(CloudType{},rotation,position,-1.),"time rollback reused model");
    GroundPlaneTracker travel(cfg);travel.update(floor,M3D::Identity(),V3D::Zero(),0.);
    require(!travel.update(CloudType{},M3D::Identity(),V3D(1,0,0),.1),"unsupported model reused after travel");
    // Ground is cleared while a 12 cm obstacle and vertical wall survive.
    KeyPoseWithCloud frame;frame.r_global=M3D::Identity();frame.t_global=V3D::Zero();frame.time=0.;
    frame.body_cloud=std::make_shared<CloudType>(floor);
    PointType obstacle;obstacle.x=2.013;obstacle.y=1.017;
    obstacle.z=.035*obstacle.x-.02*obstacle.y+.01+.12;frame.body_cloud->push_back(obstacle);
    PointType wall;wall.x=-2.013;wall.y=-1.017;wall.z=.8;frame.body_cloud->push_back(wall);
    const auto original=*frame.body_cloud;
    const auto corrected=buildOccupancy({frame},GridConfig{});
    require(value(corrected,obstacle.x,obstacle.y)==100,"low obstacle removed");
    require(value(corrected,wall.x,wall.y)==100,"wall removed");
    require(value(corrected,2.713,-2.383)==0,"sloping floor still occupied");
    require(std::count(corrected.data.begin(),corrected.data.end(),100)==2,"floor leaked into occupancy");
    for(std::size_t i=0;i<original.size();++i)
        require((original[i].getVector3fMap()-(*frame.body_cloud)[i].getVector3fMap()).norm()==0.,"raw PCD modified");
    GridConfig disabled;disabled.ground.enabled=false;
    const auto old=buildOccupancy({frame},disabled);
    require(std::count(old.data.begin(),old.data.end(),100)>50,"disabled mode did not preserve fixed slice");
    // With no trustworthy floor, fixed slicing retains possible obstacles.
    frame.body_cloud=std::make_shared<CloudType>(floorCloud(0,0,.12));
    const auto fallback=buildOccupancy({frame},GridConfig{});
    require(std::count(fallback.data.begin(),fallback.data.end(),100)>100,"unsupported surface silently cleared");
    std::cout<<"PASS sloping ground, height drift, low obstacles, walls, continuity, fallback and raw cloud preservation\n";
}
