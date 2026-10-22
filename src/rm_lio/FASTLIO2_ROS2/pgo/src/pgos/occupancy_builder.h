#pragma once
#include "simple_pgo.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <unordered_set>
#include <limits>
#include <stdexcept>

struct GridConfig {
    double resolution=.05, minimum_z=.1, maximum_z=1.5, ground_z=-.15, range=10.;
    std::size_t maximum_cells=4000000;
};

// Rebuild from immutable body-frame observations and FINAL optimized poses.
// No historical raster survives an optimization; obstacles dominate clearing.
inline nav_msgs::msg::OccupancyGrid buildOccupancy(const std::vector<KeyPoseWithCloud> &frames,
                                                  const GridConfig &cfg) {
    struct Observation { V3D origin,base; std::vector<V3D> points; };
    std::vector<Observation> observations;
    double minx=std::numeric_limits<double>::infinity(), miny=minx, maxx=-minx,maxy=-minx;
    auto bound=[&](const V3D &p){minx=std::min(minx,p.x());miny=std::min(miny,p.y());maxx=std::max(maxx,p.x());maxy=std::max(maxy,p.y());};
    for (const auto &f:frames) {
        Observation obs;obs.origin=f.r_global*V3D(0,.22157,.33018)+f.t_global;obs.base=f.t_global;
        bound(obs.origin);bound(obs.base);
        for(const auto &p:*f.body_cloud) {
            V3D point=f.r_global*p.getVector3fMap().cast<double>()+f.t_global;
            if (!point.allFinite() || point.z()<cfg.ground_z || point.z()>cfg.maximum_z ||
                (point-obs.origin).head<2>().norm()>cfg.range) continue;
            obs.points.push_back(point);bound(point);
        }
        observations.push_back(std::move(obs));
    }
    if (observations.empty()) throw std::runtime_error("No mapping observations");
    minx=std::floor(minx/cfg.resolution)*cfg.resolution-.5;miny=std::floor(miny/cfg.resolution)*cfg.resolution-.5;
    const int width=static_cast<int>(std::ceil((maxx+.5-minx)/cfg.resolution))+1;
    const int height=static_cast<int>(std::ceil((maxy+.5-miny)/cfg.resolution))+1;
    if(width<=0 || height<=0 || static_cast<std::size_t>(width)*height>cfg.maximum_cells) throw std::runtime_error("Grid exceeds maximum_cells");
    nav_msgs::msg::OccupancyGrid grid;grid.header.frame_id="map";
    grid.info.resolution=cfg.resolution;grid.info.width=width;grid.info.height=height;
    grid.info.origin.position.x=minx;grid.info.origin.position.y=miny;grid.info.origin.orientation.w=1.;
    grid.data.assign(width*height,-1);std::vector<bool> occupied(width*height,false);
    auto cell=[&](const V3D &p){return std::pair<int,int>(static_cast<int>(std::floor((p.x()-minx)/cfg.resolution)),static_cast<int>(std::floor((p.y()-miny)/cfg.resolution)));};
    auto clear=[&](int x,int y){if(x>=0&&y>=0&&x<width&&y<height)grid.data[y*width+x]=0;};
    for(const auto &obs:observations) {
        const auto start=cell(obs.origin);const auto base=cell(obs.base);clear(base.first,base.second);
        std::unordered_set<int> ends;
        for(const auto &p:obs.points) {
            const auto end=cell(p);const int idx=end.second*width+end.first;
            if(p.z()>=cfg.minimum_z)occupied[idx]=true;
            ends.insert(idx);
        }
        for(const int idx:ends) {
            int x=start.first,y=start.second,ex=idx%width,ey=idx/width;
            const int dx=std::abs(ex-x),dy=std::abs(ey-y),sx=x<ex?1:-1,sy=y<ey?1:-1;
            int error=dx-dy;
            while(true) {
                clear(x,y);if(x==ex&&y==ey)break;
                const int twice=2*error;
                if(twice>-dy){error-=dy;x+=sx;}if(twice<dx){error+=dx;y+=sy;}
            }
        }
    }
    for(std::size_t i=0;i<occupied.size();++i)if(occupied[i])grid.data[i]=100;
    return grid;
}
