#include "pgos/occupancy_builder.h"
#include <iostream>
#include <stdexcept>
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
CloudType::Ptr scene(){
 auto result=std::make_shared<CloudType>();
 for(int i=0;i<45;++i)for(int j=0;j<25;++j){PointType p;p.intensity=1;
  p.x=.11*i;p.y=.11*j;p.z=0;result->push_back(p);
  p.x=0;p.y=.11*i;p.z=.07*j;result->push_back(p);
  p.x=.11*i;p.y=4.;p.z=.07*j;result->push_back(p);
 }return result;
}
CloudWithPose scan(const CloudType::Ptr &world,double time,const V3D &truth,const V3D &measured){
 CloudWithPose c;c.pose.r=M3D::Identity();c.pose.t=measured;c.pose.setTime(static_cast<int>(time),0);
 c.cloud=std::make_shared<CloudType>();const V3D inverse=-truth;const Eigen::Quaterniond identity=Eigen::Quaterniond::Identity();pcl::transformPointCloud(*world,*c.cloud,inverse,identity);return c;
}
int value(const nav_msgs::msg::OccupancyGrid &g,double x,double y){
 const int col=std::floor((x-g.info.origin.position.x)/g.info.resolution),row=std::floor((y-g.info.origin.position.y)/g.info.resolution);
 return col<0||row<0||col>=static_cast<int>(g.info.width)||row>=static_cast<int>(g.info.height)?-1:g.data[row*g.info.width+col];
}
int main(){
 Config c;c.key_pose_delta_trans=.05;c.loop_search_radius=.85;c.loop_time_tresh=20;c.loop_min_path_length=4;
 c.min_loop_detect_duration=5;c.loop_score_tresh=.02;c.loop_confirmations=2;c.planar_graph=true;
 SimplePGO pgo(c);auto world=scene();V3D last_truth=V3D::Zero();
 for(int i=0;i<44;++i){
  V3D truth;
  if(i<10)truth=V3D(i*.3,0,0);else if(i<20)truth=V3D(3,(i-10)*.2,0);
  else if(i<30)truth=V3D(3-(i-20)*.3,2,0);else if(i<40)truth=V3D(0,2-(i-30)*.2,0);
  else truth=V3D((i-40)*.1,0,0);
  auto cp=scan(world,i+1,truth,truth+V3D(.012*i,-.003*i,0));
  if(pgo.addKeyPose(cp)){pgo.searchForLoopPairs();pgo.smoothAndUpdate();}
  last_truth=truth;
 }
 const double before=V3D(.012*43,-.003*43,0).norm();const double after=(pgo.keyPoses().back().t_global-last_truth).norm();
 std::cout<<"loops="<<pgo.historyPairs().size()<<" odom error="<<before<<" optimized error="<<after<<"\n";
 require(!pgo.historyPairs().empty(),"no confirmed loop detected");
 require(after<before*.6,"pose graph did not reduce drift");
 require(pgo.keyPoses().front().t_global.norm()<1e-5,"initial map anchor moved");
 require((pgo.keyPoses()[25].t_global-pgo.keyPoses()[25].t_local).norm()>.01,"historical poses not optimized");
 // Source largely unrelated to target: small overlapping fragment cannot make a loop.
 SimplePGO negative(c);auto unrelated=scene();for(auto &p:*unrelated){p.x+=20;p.y+=20;}
 for(int i=0;i<44;++i){const V3D truth(i<22?i*.15:(43-i)*.15,0,0);
  auto cp=scan(i<25?world:unrelated,i+1,truth,truth);if(negative.addKeyPose(cp)){negative.searchForLoopPairs();negative.smoothAndUpdate();}}
 require(negative.historyPairs().empty(),"unrelated return accepted as a loop");
 // Rebuilding optimized keyframes must remove occupancy at the OLD wall location.
 KeyPoseWithCloud f;f.r_global=M3D::Identity();f.t_global=V3D(.5,0,0);f.body_cloud=std::make_shared<CloudType>();
 PointType p;p.x=2;p.y=.5;p.z=.5;p.intensity=1;f.body_cloud->push_back(p);
 const auto old=buildOccupancy({f},GridConfig{});f.t_global.setZero();const auto corrected=buildOccupancy({f},GridConfig{});
 require(value(old,2.52,.52)==100,"test old wall missing");require(value(corrected,2.02,.52)==100,"corrected wall missing");
 require(value(corrected,2.52,.52)!=100,"old raster occupancy survived correction");
 require(value(corrected,1.02,.37)==0,"left-mounted sensor free ray missing");
 require(value(corrected,1.02,.22)==-1,"old front-mounted sensor ray cleared unobserved space");
 require(value(corrected,.1,-.3)==-1,"unobserved area cleared");
 std::cout<<"PASS confirmed loop optimization, anchor, history rebuild and unrelated-scene rejection\n";
}
