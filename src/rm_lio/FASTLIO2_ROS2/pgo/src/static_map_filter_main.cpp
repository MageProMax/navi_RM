#include "pgos/map_export.h"
#include <sstream>
#include <iostream>
#include <map>

int main(int argc,char **argv) {
    try {
        std::map<std::string,std::string> args;
        for(int i=1;i<argc;i+=2){if(i+1>=argc)throw std::runtime_error("Expected --option value");args[argv[i]]=argv[i+1];}
        const std::filesystem::path source=args.at("--maps-path"),output=args.at("--save-dir");
        if(std::filesystem::exists(output) && std::filesystem::equivalent(source,output))throw std::runtime_error("Output must differ from raw input");
        StaticFilterConfig config;
        if(args.count("--config"))config=readFilterConfig(args.at("--config"));
        else if(std::filesystem::exists(source/"dynamic_filter.yaml"))config=readFilterConfig(source/"dynamic_filter.yaml");
        std::ifstream poses(source/"poses.txt"),times(source/"frame_times.txt"),override;
        if(!poses)throw std::runtime_error("Cannot read poses.txt");
        if(args.count("--poses")){override.open(args.at("--poses"));if(!override)throw std::runtime_error("Cannot read pose override");}
        std::vector<KeyPoseWithCloud> frames;std::string line;
        const auto patches=std::filesystem::canonical(source/"patches");
        while(std::getline(poses,line)) {
            if(line.empty())continue;
            std::istringstream row(line);std::string name;double tx,ty,tz,qw,qx,qy,qz;
            if(!(row>>name>>tx>>ty>>tz>>qw>>qx>>qy>>qz))throw std::runtime_error("Invalid eight-column PGO pose");
            std::string extra;if(row>>extra)throw std::runtime_error("Extra PGO pose column");
            if(override.is_open() && !(override>>tx>>ty>>tz>>qw>>qx>>qy>>qz))throw std::runtime_error("Refined pose count mismatch");
            Eigen::Quaterniond q(qw,qx,qy,qz);V3D position(tx,ty,tz);
            if(!q.coeffs().allFinite() || q.norm()<1e-8 || !position.allFinite())throw std::runtime_error("Invalid pose");
            const auto path=std::filesystem::canonical(patches/name);
            const auto relative=path.lexically_relative(patches);
            if(relative.empty() || *relative.begin()=="..")throw std::runtime_error("Patch outside input directory");
            KeyPoseWithCloud f;f.r_global=q.normalized().toRotationMatrix();f.t_global=position;
            f.r_local=f.r_global;f.t_local=f.t_global;f.time=frames.size()*config.min_observation_interval;
            if(times.is_open() && !(times>>f.time))throw std::runtime_error("Timestamp count mismatch");
            f.body_cloud=std::make_shared<CloudType>();
            if(pcl::io::loadPCDFile(path.string(),*f.body_cloud)<0)throw std::runtime_error("Cannot load patch "+name);
            frames.push_back(f);
        }
        double extra;if(override.is_open() && override>>extra)throw std::runtime_error("Extra refined pose rows");
        if(times.is_open() && times>>extra)throw std::runtime_error("Extra timestamps");
        if(frames.empty())throw std::runtime_error("No observations");
        auto filtered=filterStaticMap(frames,config);
        std::filesystem::create_directories(output);writePatches(output,filtered.frames);
        writeFilterMetadata(output,config,filtered.stats);
        std::cout<<"Static-map filter: "<<filtered.stats.state<<", removed "<<filtered.stats.removed_points<<" / "<<filtered.stats.input_points<<" points\n";
        return 0;
    }catch(const std::exception &e){std::cerr<<"Static-map filter failed: "<<e.what()<<'\n';return 1;}
}
