#include "map_builder/imu_processor.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <limits>

static void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}

static SyncPackage samples(const V3D &acc,const V3D &gyro,double begin,double end) {
    SyncPackage package;
    for(int i=0;i<=static_cast<int>(std::round((end-begin)*200));++i) {
        double time=begin+i/200.;package.imus.emplace_back(acc,gyro,time);
    }
    package.cloud_start_time=begin;package.cloud_end_time=end;
    return package;
}

int main() {
    Config cfg;cfg.imu_init_num=200;cfg.imu_init_duration=5.;cfg.gravity_align=true;
    cfg.r_il=M3D::Identity();cfg.t_il=V3D(-.011,-.02329,.04412);
    const M3D installed=Eigen::AngleAxisd(-std::acos(-1.)/4.,V3D::UnitX()).toRotationMatrix();
    const V3D up_in_imu=installed.transpose()*V3D::UnitZ();
    const V3D bias(.001,-.002,.003);
    for(double native_gravity:{1.,9.81}) {
        auto filter=std::make_shared<IESKF>();IMUProcessor processor(cfg,filter);
        auto short_window=samples(up_in_imu*native_gravity,bias,0.,4.9);
        require(!processor.initialize(short_window),"IMU accepted less than five stationary seconds");
        auto end=samples(up_in_imu*native_gravity,bias,4.905,5.1);
        require(processor.initialize(end),"45 degree installed stationary IMU failed initialization");
        require((filter->x().r_wi*up_in_imu-V3D::UnitZ()).norm()<1e-10,"incorrect gravity direction for tilted internal IMU");
        require((filter->x().bg-bias).norm()<1e-10,"gyro zero bias not initialized");
        require((filter->x().r_il-cfg.r_il).norm()<1e-10&&(filter->x().t_il-cfg.t_il).norm()<1e-10,"internal LiDAR/IMU extrinsic changed");
        require((filter->x().g+V3D(0,0,9.81)).norm()<1e-10,"gravity sign or magnitude wrong");
        auto scan=samples(up_in_imu*native_gravity,bias,5.105,5.2);
        scan.cloud=std::make_shared<CloudType>();PointType point{};
        point.x=1.;point.y=.3;point.z=.2;point.curvature=0.;scan.cloud->push_back(point);
        scan.cloud_start_time=5.1;scan.cloud_end_time=5.2;
        processor.undistort(scan);
        require(filter->x().v.norm()<1e-9,"native g or SI acceleration was incorrectly scaled");
        require((filter->x().r_wi*up_in_imu-V3D::UnitZ()).norm()<1e-9,"stationary propagation changed gravity alignment");
    }
    auto rejects=[&](SyncPackage package) {
        auto filter=std::make_shared<IESKF>();IMUProcessor processor(cfg,filter);
        return !processor.initialize(package);
    };
    require(rejects(samples(up_in_imu,V3D(.1,0,0),0.,5.1)),"rotating IMU passed stationary gate");
    auto noise=samples(up_in_imu,V3D::Zero(),0.,5.1);
    for(std::size_t i=0;i<noise.imus.size();++i)noise.imus[i].acc.x()+=(i%2?.2:-.2);
    require(rejects(noise),"strong vibration passed stationary gate");
    require(rejects(samples(V3D::Zero(),V3D::Zero(),0.,5.1)),"zero acceleration accepted");
    auto interrupted=samples(up_in_imu,V3D::Zero(),0.,1.);
    auto resumed=samples(up_in_imu,V3D::Zero(),4.1,5.1);
    interrupted.imus.insert(interrupted.imus.end(),resumed.imus.begin(),resumed.imus.end());
    require(rejects(interrupted),"missing IMU seconds counted as continuous stationary initialization");
    auto filter_after_gap=std::make_shared<IESKF>();IMUProcessor after_gap(cfg,filter_after_gap);
    require(!after_gap.initialize(interrupted),"interrupted window accepted");
    auto continuous=samples(up_in_imu,V3D::Zero(),5.105,9.2);
    require(after_gap.initialize(continuous),"initialization did not recover after five fresh seconds");
    auto rollback=samples(up_in_imu,V3D::Zero(),0.,5.1);
    rollback.imus[500].time=1.;
    require(rejects(rollback),"out-of-order time did not restart initialization");
    auto nonfinite=samples(up_in_imu,V3D::Zero(),0.,5.1);
    nonfinite.imus[500].acc.x()=std::numeric_limits<double>::quiet_NaN();
    auto valid_after_bad=samples(up_in_imu,V3D::Zero(),5.105,6.);
    nonfinite.imus.insert(nonfinite.imus.end(),valid_after_bad.imus.begin(),valid_after_bad.imus.end());
    require(rejects(nonfinite),"nonfinite sample did not restart the stationary window");
    // A stable transverse accelerometer offset is not observable from one
    // stationary orientation. The scale normalization cannot remove this tilt.
    auto filter=std::make_shared<IESKF>();IMUProcessor processor(cfg,filter);
    const double tilt=.026;
    auto biased=samples(installed.transpose()*V3D(tilt,0,1),V3D::Zero(),0.,5.1);
    require(processor.initialize(biased),"stable biased sample rejected unexpectedly");
    const double observed=std::acos(std::clamp((filter->x().r_wi*up_in_imu).dot(V3D::UnitZ()),-1.,1.));
    require(std::abs(observed-std::atan(tilt))<1e-8,"bias/tilt diagnostic does not match geometry");
    std::cout<<"PASS native g/SI scaling, 45 degree installation, five-second window, gyro bias, vibration gates; "
             <<"stable 0.026 g transverse bias gives "<<observed*180./std::acos(-1.)<<" degrees (simulation only)\n";
}
