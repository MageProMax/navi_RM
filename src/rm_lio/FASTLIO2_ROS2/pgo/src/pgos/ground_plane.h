#pragma once
#include "commons.h"
#include <map>
#include <optional>
#include <random>
#include <cmath>
#include <stdexcept>

struct GroundPlaneConfig {
    bool enabled=true;
    double radius=4., max_slope_deg=12., max_offset=.35, max_step=.08;
    double threshold=.035, max_age=1., max_travel=.5;
    int minimum_points=30;
};

inline void validateGroundPlane(const GroundPlaneConfig &c) {
    for(double v:{c.radius,c.max_slope_deg,c.max_offset,c.max_step,c.threshold,c.max_age,c.max_travel})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid ground-plane limits");
    if(c.max_slope_deg>=45||c.minimum_points<8||c.max_step>c.max_offset)
        throw std::invalid_argument("Invalid ground-plane support/continuity");
}

// The cloud is already leveled by LIO. Fit only the raster's height reference;
// never move raw PCD points or change the optimized XY poses.
class GroundPlaneTracker {
public:
    explicit GroundPlaneTracker(const GroundPlaneConfig &config):cfg_(config) {validateGroundPlane(cfg_);}
    std::optional<V3D> update(const CloudType &cloud,const M3D &rotation,
                              const V3D &position,double time) {
        if(!cfg_.enabled)return std::nullopt;
        if(have_model_&&(!std::isfinite(time)||time<last_time_))have_model_=false;
        // Seeding a new supported fit tolerates ordinary keyframe gaps. Reuse
        // without fresh support has much tighter time and travel bounds.
        const double travel=(position-last_position_).head<2>().norm();
        const bool continuous=have_model_&&std::isfinite(time)&&time>=last_time_&&
            time-last_time_<=30.&&travel<=cfg_.radius;
        const bool recent=continuous&&time-last_time_<=cfg_.max_age&&travel<=cfg_.max_travel;
        V3D expected=V3D::Zero();
        if(continuous) {
            expected.head<2>()=rotation.topLeftCorner<2,2>().transpose()*world_plane_.head<2>();
            expected.z()=world_plane_.head<2>().dot(position.head<2>())+world_plane_.z()-position.z();
        }
        const auto plane=fit(cloud,expected.z());
        if(plane&&std::isfinite(time)) {
            world_plane_.head<2>()=rotation.topLeftCorner<2,2>()*plane->head<2>();
            world_plane_.z()=plane->z()+position.z()-world_plane_.head<2>().dot(position.head<2>());
            last_time_=time;last_position_=position;have_model_=true;
            return plane;
        }
        // Unsupported/expired models fall back to the original fixed slice.
        return recent?std::optional<V3D>(expected):std::nullopt;
    }
private:
    std::optional<V3D> fit(const CloudType &cloud,double expected_offset) const {
        std::map<std::pair<int,int>,V3D> cells;
        for(const auto &p:cloud) {
            const V3D v=p.getVector3fMap().cast<double>();
            if(!v.allFinite()||v.head<2>().norm()>cfg_.radius||std::abs(v.z())>cfg_.max_offset+.30||
               (std::abs(v.x())<.38&&std::abs(v.y())<.33))continue;
            const auto key=std::make_pair(static_cast<int>(std::floor(v.x()/.15)),static_cast<int>(std::floor(v.y()/.15)));
            const auto it=cells.find(key);
            if(it==cells.end()||v.z()<it->second.z())cells[key]=v;
        }
        if(cells.size()<static_cast<std::size_t>(cfg_.minimum_points))return std::nullopt;
        std::vector<V3D> seeds;
        // Bound per-frame fit cost even in dense near-field clouds.
        const std::size_t stride=std::max<std::size_t>(1,(cells.size()+1999)/2000);
        std::size_t index=0;
        for(const auto &entry:cells)if(index++%stride==0)seeds.push_back(entry.second);
        const std::size_t minimum=std::max<std::size_t>(cfg_.minimum_points,std::ceil(.35*seeds.size()));
        const double slope=std::tan(cfg_.max_slope_deg*std::acos(-1.)/180.);
        auto allowed=[&](const V3D &p){return p.allFinite()&&p.head<2>().norm()<=slope&&
            std::abs(p.z())<=cfg_.max_offset&&std::abs(p.z()-expected_offset)<=cfg_.max_step;};
        auto residual=[](const V3D &p,const V3D &v){return v.z()-p.x()*v.x()-p.y()*v.y()-p.z();};
        std::mt19937 random(0);std::uniform_int_distribution<std::size_t> pick(0,seeds.size()-1);
        V3D best=V3D::Zero();std::size_t best_count=0;
        for(int iteration=0;iteration<80;++iteration) {
            const auto i=pick(random),j=pick(random),k=pick(random);
            if(i==j||i==k||j==k)continue;
            M3D a;a<<seeds[i].x(),seeds[i].y(),1.,seeds[j].x(),seeds[j].y(),1.,seeds[k].x(),seeds[k].y(),1.;
            if(std::abs(a.determinant())<1e-8)continue;
            const V3D plane=a.inverse()*V3D(seeds[i].z(),seeds[j].z(),seeds[k].z());
            if(!allowed(plane))continue;
            std::size_t count=0;for(const auto &v:seeds)if(std::abs(residual(plane,v))<=cfg_.threshold)++count;
            if(count>best_count){best=plane;best_count=count;}
        }
        if(best_count<minimum)return std::nullopt;
        for(int iteration=0;iteration<2;++iteration) {
            M3D normal=M3D::Zero();V3D rhs=V3D::Zero();std::size_t count=0;
            for(const auto &v:seeds)if(std::abs(residual(best,v))<=cfg_.threshold) {
                const V3D row(v.x(),v.y(),1.);normal+=row*row.transpose();rhs+=row*v.z();++count;
            }
            if(count<minimum||std::abs(normal.determinant())<1e-8)return std::nullopt;
            best=normal.ldlt().solve(rhs);
        }
        if(!allowed(best))return std::nullopt;
        Eigen::Vector2d sum=Eigen::Vector2d::Zero();Eigen::Matrix2d squares=Eigen::Matrix2d::Zero();
        std::size_t count=0,near=0;
        for(const auto &v:seeds)if(std::abs(residual(best,v))<=cfg_.threshold) {
            ++count;if(v.head<2>().norm()<=1.5)++near;
            sum+=v.head<2>();squares+=v.head<2>()*v.head<2>().transpose();
        }
        if(count<minimum||near<8)return std::nullopt;
        const Eigen::Matrix2d covariance=(squares-sum*sum.transpose()/count)/(count-1);
        if(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(covariance).eigenvalues().minCoeff()<.02)return std::nullopt;
        return best;
    }
    GroundPlaneConfig cfg_;
    bool have_model_=false;
    V3D world_plane_=V3D::Zero(),last_position_=V3D::Zero();
    double last_time_=0.;
};
