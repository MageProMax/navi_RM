#include "localizers/icp_localizer.h"
#include "localizers/scan_history.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <unistd.h>

void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
M4F pose(float x, float y, float yaw) {
    M4F p=M4F::Identity(); p.block<3,3>(0,0)=Eigen::AngleAxisf(yaw,V3F::UnitZ()).toRotationMatrix();
    p(0,3)=x; p(1,3)=y; return p;
}
int main() {
    auto world=std::make_shared<CloudType>();
    // Three perpendicular, unevenly sized surfaces constrain all six axes.
    for(int i=0;i<35;++i) for(int j=0;j<25;++j) {
        PointType p; p.intensity=1;
        p.x=.09f*i; p.y=.085f*j; p.z=.003f*std::sin(i+j); world->push_back(p);
        p.x=0; p.y=.085f*j; p.z=.07f*i; world->push_back(p);
        p.x=.09f*i; p.y=2.2f; p.z=.07f*j; world->push_back(p);
    }
    ScanHistory history;
    for(int i=0;i<4;++i) {
        const auto t=pose(.1f*i,.03f*i,.1f*i);
        auto scan=std::make_shared<CloudType>(); pcl::transformPointCloud(*world,*scan,t.inverse());
        history.push({1.+i*.1,t,scan},.5,10);
    }
    const auto accumulated=history.latestBodyCloud();
    CloudType expected; pcl::transformPointCloud(*world,expected,history.scans.back().odom_body.inverse());
    require(accumulated->size()==world->size()*4,"history lost scans");
    for(std::size_t i=0;i<accumulated->size();++i)
        require(((*accumulated)[i].getVector3fMap()-expected[i%expected.size()].getVector3fMap()).norm()<1e-4,
                "moving/rotating history is blurred");
    history.push({1.4,pose(.4,.1,.4),world},.15,2);
    require(history.scans.size()==2,"history time/count not bounded");
    history.push({.2,pose(0,0,0),world},.5,10);
    require(history.scans.size()==1,"clock rollback not reset");
    history.push({.3,pose(3,0,0),world},.5,10);
    require(history.scans.size()==1,"odometry jump not reset");
    const auto path=std::filesystem::temp_directory_path()/ ("registration_test_"+std::to_string(getpid())+".pcd");
    pcl::io::savePCDFileBinary(path.string(),*world);
    for(const auto method : {"icp","gicp"}) {
        ICPConfig config; config.registration_method=method;
        config.rough_max_iteration=50; config.refine_max_iteration=50;
        config.rough_scan_resolution=.08; config.rough_map_resolution=.08;
        config.refine_scan_resolution=.04; config.refine_map_resolution=.04;
        ICPLocalizer localizer(config); require(localizer.loadMap(path.string()),"map failed");
        auto source=std::make_shared<CloudType>(); const auto truth=pose(.3,-.2,.08);
        pcl::transformPointCloud(*world,*source,truth.inverse());
        localizer.setInput(source); auto estimate=pose(.2,-.1,.03);
        require(localizer.align(estimate),"known transform rejected");
        const float error=(estimate.block<3,1>(0,3)-truth.block<3,1>(0,3)).norm();
        require(error<.04,"translation accuracy regression");
        require((estimate.block<3,3>(0,0)-truth.block<3,3>(0,0)).norm()<.04,"rotation accuracy regression");
        std::cout<<method<<" synthetic translation error="<<error<<" overlap="<<localizer.inlierRatio()<<"\n";
        // Mostly unrelated returns must not pass on a tiny overlapping subset.
        auto partial=std::make_shared<CloudType>(*source);
        for(int i=0;i<10000;++i) { PointType p; p.x=20+(i%100)*.1f; p.y=20+(i/100)*.1f; p.z=1; p.intensity=1; partial->push_back(p); }
        localizer.setInput(partial); estimate=truth;
        require(!localizer.align(estimate),"low-overlap scan accepted");
        auto empty=std::make_shared<CloudType>(); localizer.setInput(empty);
        require(!localizer.align(estimate),"empty scan accepted");
    }
    std::filesystem::remove(path);
    std::cout<<"PASS motion compensation, history reset/bounds, ICP/GICP pose recovery, overlap rejection\n";
}
