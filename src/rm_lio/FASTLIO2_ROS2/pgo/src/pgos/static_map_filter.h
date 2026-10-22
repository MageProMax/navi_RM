#pragma once
#include "simple_pgo.h"
#include <cstddef>
#include <string>

// Static-map evidence only. Never apply this filter to live collision scans.
struct StaticFilterConfig {
    bool enabled = true;
    double voxel_size = .10;
    double endpoint_margin = .20;
    double min_observation_interval = .5;
    int min_free_observations = 3;
    double free_to_hit_ratio = 3.;
    double max_range = 10.;
    std::size_t maximum_voxels = 2000000;
    std::size_t maximum_ray_steps = 60000000;
};
struct StaticFilterStats {
    std::size_t input_points=0, removed_points=0, removed_voxels=0, candidate_voxels=0, ray_steps=0;
    std::string state="disabled";
};
struct StaticFilterResult {
    std::vector<KeyPoseWithCloud> frames;
    StaticFilterStats stats;
};
void validateStaticFilter(const StaticFilterConfig &config);
StaticFilterResult filterStaticMap(const std::vector<KeyPoseWithCloud> &frames,
                                  const StaticFilterConfig &config);
