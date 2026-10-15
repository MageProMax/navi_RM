#pragma once
#include "commons.h"
#include <deque>
#include <algorithm>
#include <cmath>
#include <pcl/common/transforms.h>

struct TimedScan {
    double stamp;
    M4F odom_body;
    CloudType::ConstPtr cloud;
};

// Caller owns synchronization. Stored scans are immutable and bounded in time/count.
class ScanHistory {
public:
    void clear() { scans.clear(); }
    void push(const TimedScan &scan, double seconds, std::size_t maximum) {
        if (!scans.empty()) {
            const auto &last = scans.back();
            const M4F delta = last.odom_body.inverse() * scan.odom_body;
            const float cosine = std::clamp((delta.block<3,3>(0,0).trace()-1.f)/2.f,-1.f,1.f);
            if (scan.stamp <= last.stamp || scan.stamp-last.stamp > seconds+0.5 ||
                delta.block<3,1>(0,3).norm() > 1.f || std::acos(cosine) > 0.785398f) clear();
        }
        scans.push_back(scan);
        while (scans.size() > 1 && (scans.size() > maximum || scan.stamp-scans.front().stamp > seconds))
            scans.pop_front();
    }
    CloudType::Ptr latestBodyCloud() const {
        auto result = std::make_shared<CloudType>();
        if (scans.empty()) return result;
        const M4F body_odom = scans.back().odom_body.inverse();
        for (const auto &scan : scans) {
            CloudType transformed;
            pcl::transformPointCloud(*scan.cloud, transformed, body_odom * scan.odom_body);
            *result += transformed;
        }
        return result;
    }
    std::deque<TimedScan> scans;
};
