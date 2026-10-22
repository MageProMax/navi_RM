#pragma once
#include "commons.h"

// Re-express physical-body points in the published planar navigation body.
// The initial base height defines z=0; subsequent heave/roll/pitch must live
// in the cloud when they are deliberately omitted from odom->base_link.
inline void planarCloudTransform(const M3D &r_full,const V3D &t_full,
                                 const M3D &r_planar,const V3D &t_planar,
                                 double initial_base_z,M3D &rotation,V3D &translation)
{
    rotation=r_planar.transpose()*r_full;
    translation=r_planar.transpose()*(t_full-t_planar-V3D(0.,0.,initial_base_z));
}
