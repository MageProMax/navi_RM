#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

struct TrackingCorrection
{
    bool accepted = false;
    double translation = std::numeric_limits<double>::infinity();
    double rotation = std::numeric_limits<double>::infinity();
};

// Compare to the odometry prediction for THIS scan, not the preceding scan.
// Initial/manual relocalization can deliberately establish a distant pose.
inline TrackingCorrection checkTrackingCorrection(
    const Eigen::Matrix4f &prediction, const Eigen::Matrix4f &candidate,
    double max_translation, double max_rotation, bool relocalizing)
{
    TrackingCorrection result;
    if (!prediction.allFinite() || !candidate.allFinite())
        return result;
    result.translation = (candidate.block<3, 1>(0, 3) -
                          prediction.block<3, 1>(0, 3)).norm();
    const Eigen::Matrix3f relative_rotation =
        prediction.block<3, 3>(0, 0).transpose() * candidate.block<3, 3>(0, 0);
    result.rotation = std::abs(Eigen::AngleAxisf(relative_rotation).angle());
    result.accepted = std::isfinite(result.rotation) &&
        (relocalizing || (result.translation <= max_translation &&
                         result.rotation <= max_rotation));
    return result;
}
