#include "map_builder/ieskf.h"
#include "map_builder/planar_projection.h"
#include <iostream>
#include <stdexcept>

static void require(bool condition,const char *message)
{if(!condition)throw std::runtime_error(message);}

int main()
{
    IESKF filter;filter.P()=M21D::Identity()*.001;
    const M21D before=filter.P();
    filter.setLossFunction([](State &,SharedState &d){d.valid=false;});
    filter.setStopFunction([](const V21D &){return true;});
    require(!filter.update(),"Invalid scan accepted");
    require((filter.P()-before).norm()<1e-14,"Missing scan reset covariance");
    filter.setLossFunction([](State &,SharedState &d){d.valid=true;d.H=M12D::Identity()*100.;d.b=V12D::Zero();d.b(3)=-1.;});
    require(filter.update(),"Valid scan rejected");
    require(filter.x().t_wi.x()>0. && filter.P()(3,3)<before(3,3),"Valid observation did not update position and covariance");
    filter.x().t_wi.setZero();filter.P()=before;filter.setStopFunction([](const V21D &){return false;});
    filter.setLossFunction([](State &,SharedState &d){d.valid=d.iter_num==0;d.H=M12D::Identity()*100.;d.b=V12D::Zero();d.b(3)=-1.;});
    require(!filter.update(),"Partially invalid iteration accepted");
    require(filter.x().t_wi.norm()<1e-14 && (filter.P()-before).norm()<1e-14,"Failed iteration did not roll back to IMU prediction");
    // A level world point seen by a pitched, heaving body must reconstruct
    // exactly with the planar TF, including the initial height reference.
    const M3D full=(Eigen::AngleAxisd(.4,V3D::UnitZ())*Eigen::AngleAxisd(.12,V3D::UnitY())*Eigen::AngleAxisd(-.08,V3D::UnitX())).toRotationMatrix();
    const M3D planar=Eigen::AngleAxisd(.4,V3D::UnitZ()).toRotationMatrix();
    const V3D t_full(2.,3.,-.25),t_planar(2.,3.,0.),ground_world(4.,5.,-.30);
    const V3D body=full.transpose()*(ground_world-t_full);
    M3D r;V3D t;planarCloudTransform(full,t_full,planar,t_planar,-.30,r,t);
    const V3D reconstructed=planar*(r*body+t)+t_planar;
    require((reconstructed-V3D(4.,5.,0.)).norm()<1e-12,"Planar cloud/TF projection lost the level ground point");
    std::cout<<"PASS invalid covariance, valid update, failed iteration rollback, tilted/heaving body projection\n";
}
