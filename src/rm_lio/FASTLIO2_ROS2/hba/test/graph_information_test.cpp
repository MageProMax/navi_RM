#include "hba/graph_information.h"
#include <limits>
#include <iostream>
#include <stdexcept>

int main()
{
    M6D cross=M6D::Zero();cross.diagonal()<<-4.,-3.,-2.,-10.,-20.,-30.;cross(0,1)=100.;
    const auto information=relativeInformation(cross);
    if(information.llt().info()!=Eigen::Success || information(0,0)!=4. || information(0,1)!=0.)
        throw std::runtime_error("Signed Hessian cross block did not produce SPD weights");
    cross.setZero();
    if(relativeInformation(cross)(0,0)!=1e-3)throw std::runtime_error("Weak direction floor missing");
    cross(1,1)=std::numeric_limits<double>::quiet_NaN();
    bool rejected=false;try{relativeInformation(cross);}catch(const std::runtime_error &){rejected=true;}
    if(!rejected)throw std::runtime_error("Non-finite Hessian accepted");
    std::cout<<"PASS signed/non-symmetric block, weak direction floor, non-finite rejection\n";
}
