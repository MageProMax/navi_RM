#pragma once
#include "commons.h"
#include <algorithm>
#include <stdexcept>

// A Hessian cross block is signed and need not be SPD. It is not a Gaussian
// information matrix. Use bounded absolute diagonal weights, as in upstream
// HBA's pose-graph approximation, retaining weak directions with low weight.
inline M6D relativeInformation(const M6D &cross)
{
    if(!cross.allFinite())throw std::runtime_error("Non-finite HBA Hessian");
    M6D information=M6D::Zero();
    for(int i=0;i<6;++i)information(i,i)=std::clamp(std::abs(cross(i,i)),1e-3,1e6);
    return information;
}
