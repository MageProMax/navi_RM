#include "static_map_filter.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace {
struct Cell {
    int x,y,z;
    bool operator==(const Cell &b) const {return x==b.x && y==b.y && z==b.z;}
};
struct Hash {
    std::size_t operator()(const Cell &k) const {
        std::size_t h=std::hash<int>{}(k.x);
        h^=std::hash<int>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2);
        return h^(std::hash<int>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2));
    }
};
struct Evidence {std::size_t hits=0,free=0;};
struct Bounds {V3D low,high;};
using Cells=std::unordered_set<Cell,Hash>;
using EvidenceMap=std::unordered_map<Cell,Evidence,Hash>;
Cell cell(const V3D &p,double resolution) {
    if(!p.allFinite() || (p.array().abs()>1e6).any()) throw std::runtime_error("Invalid mapping coordinate");
    return {static_cast<int>(std::floor(p.x()/resolution)),static_cast<int>(std::floor(p.y()/resolution)),static_cast<int>(std::floor(p.z()/resolution))};
}
V3D origin(const KeyPoseWithCloud &f) {return f.r_global*V3D(0,.22157,.33018)+f.t_global;}
V3D world(const KeyPoseWithCloud &f,const PointType &p) {return f.r_global*p.getVector3fMap().cast<double>()+f.t_global;}
bool usable(const V3D &p,const V3D &o,const StaticFilterConfig &c) {
    return p.allFinite() && (p-o).norm()<=c.max_range;
}
// Exact 3-D voxel traversal: only traversed segments with meaningful length
// count as free. Endpoint protection and no carving behind measured returns.
bool ray(const V3D &o,const V3D &p,const StaticFilterConfig &cfg,
         const EvidenceMap &evidence,Cells &free,StaticFilterStats &stats) {
    const V3D delta=p-o;const double length=delta.norm();
    if(length<=2*cfg.endpoint_margin)return true;
    const V3D direction=delta/length;
    const V3D start=o+direction*cfg.endpoint_margin;
    const double limit=length-2*cfg.endpoint_margin;
    Cell current=cell(start,cfg.voxel_size);
    std::array<int,3> indices{{current.x,current.y,current.z}},step{};
    std::array<double,3> next{},stride{};
    for(int a=0;a<3;++a) {
        if(std::abs(direction[a])<1e-12) {next[a]=stride[a]=std::numeric_limits<double>::infinity();continue;}
        step[a]=direction[a]>0?1:-1;
        const double boundary=(indices[a]+(step[a]>0?1:0))*cfg.voxel_size;
        next[a]=std::max(0.,(boundary-start[a])/direction[a]);
        stride[a]=cfg.voxel_size/std::abs(direction[a]);
    }
    double entry=0.;
    while(entry<limit) {
        if(++stats.ray_steps>cfg.maximum_ray_steps)return false;
        const double exit=std::min({next[0],next[1],next[2],limit});
        const Cell key{indices[0],indices[1],indices[2]};
        if(exit-entry>=.25*cfg.voxel_size && evidence.count(key))free.insert(key);
        if(exit>=limit)break;
        for(int a=0;a<3;++a)if(next[a]<=exit+1e-10){indices[a]+=step[a];next[a]+=stride[a];}
        entry=exit;
    }
    return true;
}
}

void validateStaticFilter(const StaticFilterConfig &c) {
    if(!std::isfinite(c.voxel_size)||c.voxel_size<.03||c.voxel_size>1.||
       !std::isfinite(c.endpoint_margin)||c.endpoint_margin<c.voxel_size||c.endpoint_margin>1.||
       !std::isfinite(c.min_observation_interval)||c.min_observation_interval<=0||
       c.min_free_observations<2||!std::isfinite(c.free_to_hit_ratio)||c.free_to_hit_ratio<2.||
       !std::isfinite(c.max_range)||c.max_range<=2*c.endpoint_margin||
       c.maximum_voxels<100||c.maximum_ray_steps<100)
        throw std::invalid_argument("Invalid static-map consistency filter configuration");
}
StaticFilterResult filterStaticMap(const std::vector<KeyPoseWithCloud> &frames,const StaticFilterConfig &c) {
    validateStaticFilter(c);
    StaticFilterResult result;result.frames=frames;
    for(const auto &f:frames) {
        if(!f.body_cloud || !std::isfinite(f.time))throw std::runtime_error("Invalid mapping observation");
        result.stats.input_points+=f.body_cloud->size();
    }
    if(!c.enabled || frames.empty())return result;
    EvidenceMap evidence;
    for(const auto &f:frames) {
        const V3D o=origin(f);
        for(const auto &p:*f.body_cloud) {
            const V3D v=world(f,p);if(!usable(v,o,c))continue;
            evidence.try_emplace(cell(v,c.voxel_size));
            if(evidence.size()>c.maximum_voxels){result.stats.state="voxel_budget_retained_all";return result;}
        }
    }
    result.stats.candidate_voxels=evidence.size();
    // Keyframes in one time bin provide at most ONE observation. Density and
    // high sensor frequency cannot manufacture independent clearing evidence.
    std::vector<std::size_t> order(frames.size());
    for(std::size_t i=0;i<order.size();++i)order[i]=i;
    std::stable_sort(order.begin(),order.end(),[&](auto a,auto b){return frames[a].time<frames[b].time;});
    for(std::size_t begin=0;begin<order.size();) {
        const double time=frames[order[begin]].time;
        if(!std::isfinite(time))throw std::runtime_error("Invalid keyframe timestamp");
        std::size_t end=begin+1;
        while(end<order.size() && frames[order[end]].time-time<c.min_observation_interval)++end;
        Cells hits,free;
        std::unordered_map<Cell,Bounds,Hash> surfaces;
        for(std::size_t i=begin;i<end;++i) {
            const auto &f=frames[order[i]];const V3D o=origin(f);
            for(const auto &p:*f.body_cloud) {
                const V3D v=world(f,p);if(!usable(v,o,c))continue;
                const Cell k=cell(v,c.voxel_size);hits.insert(k);
                auto entry=surfaces.try_emplace(k,Bounds{v,v});
                if(!entry.second){entry.first->second.low=entry.first->second.low.cwiseMin(v);entry.first->second.high=entry.first->second.high.cwiseMax(v);}
            }
        }
        for(std::size_t i=begin;i<end;++i) {
            const auto &f=frames[order[i]];const V3D o=origin(f);Cells traced;
            for(const auto &p:*f.body_cloud) {
                const V3D v=world(f,p);if(!usable(v,o,c) || !traced.insert(cell(v,c.voxel_size)).second)continue;
                if(!ray(o,v,c,evidence,free,result.stats)) {
                    result.stats.state="ray_budget_retained_all";return result;
                }
            }
        }
        for(const auto &k:hits)++evidence.at(k).hits;
        for(const auto &k:free) {
            // Protect nearby ACTUAL surfaces rather than the entire adjacent
            // voxel: a ground return must not shield a ghost 20 cm above it.
            const V3D low=V3D(k.x,k.y,k.z)*c.voxel_size;
            const V3D high=low+V3D::Constant(c.voxel_size);
            bool surface=false;
            for(int x=-1;x<=1 && !surface;++x)for(int y=-1;y<=1 && !surface;++y)for(int z=-1;z<=1;++z) {
                const auto neighbor=surfaces.find({k.x+x,k.y+y,k.z+z});
                if(neighbor==surfaces.end())continue;
                const V3D gap=(neighbor->second.low-high).cwiseMax(low-neighbor->second.high).cwiseMax(V3D::Zero());
                if(gap.norm()<=.5*c.voxel_size){surface=true;break;}
            }
            if(!surface)++evidence.at(k).free;
        }
        begin=end;
    }
    Cells removed;
    for(const auto &kv:evidence)if(kv.second.free>=static_cast<std::size_t>(c.min_free_observations) &&
        kv.second.free>=c.free_to_hit_ratio*kv.second.hits)removed.insert(kv.first);
    result.stats.removed_voxels=removed.size();
    for(std::size_t i=0;i<frames.size();++i) {
        const auto &f=frames[i];const V3D o=origin(f);CloudType::Ptr kept(new CloudType);
        for(const auto &p:*f.body_cloud) {
            const V3D v=world(f,p);
            if(usable(v,o,c) && removed.count(cell(v,c.voxel_size)))++result.stats.removed_points;
            else kept->push_back(p);
        }
        if(kept->size()!=f.body_cloud->size())result.frames[i].body_cloud=kept;
    }
    result.stats.state="filtered";return result;
}
