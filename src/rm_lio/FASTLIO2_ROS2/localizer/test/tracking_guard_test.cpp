#include "localizers/tracking_guard.h"
#include <iostream>
#include <stdexcept>

Eigen::Matrix4f pose(float x, float y, float yaw)
{
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = Eigen::AngleAxisf(
        yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    result(0, 3) = x;
    result(1, 3) = y;
    return result;
}

void require(bool condition, const char *message)
{
    if (!condition)
        throw std::runtime_error(message);
}

int main()
{
    constexpr double angle_limit = 0.1745329252;
    const auto origin = pose(0, 0, 0);
    const auto check = [&](const Eigen::Matrix4f &prediction,
                           const Eigen::Matrix4f &candidate, bool initial = false) {
        return checkTrackingCorrection(prediction, candidate, 0.30, angle_limit, initial);
    };
    require(check(origin, pose(0.1, 0.1, 0.05)).accepted, "small correction rejected");
    require(!check(origin, pose(1.0, 0, 0)).accepted, "translation jump accepted");
    require(!check(origin, pose(0, 0, 0.5)).accepted, "rotation jump accepted");
    // Moving several metres is legitimate if LIO predicts that movement.
    require(check(pose(5, 3, 1), pose(5.1, 3.05, 1.02)).accepted,
            "normal odometry motion treated as ICP jump");
    require(check(pose(0, 0, 3.13), pose(0, 0, -3.13)).accepted,
            "yaw wrapping treated as full rotation");
    require(check(origin, pose(10, 2, 1.5), true).accepted,
            "manual/initial relocalization cannot recover distant pose");
    auto invalid = origin;
    invalid(0, 3) = std::numeric_limits<float>::quiet_NaN();
    require(!check(origin, invalid, true).accepted, "invalid relocalization accepted");
    auto tilted = origin;
    tilted.block<3, 3>(0, 0) = Eigen::AngleAxisf(
        0.4, Eigen::Vector3f::UnitX()).toRotationMatrix();
    require(!check(origin, tilted).accepted, "roll jump accepted");
    std::cout << "8 tracking correction regressions passed\n";
}
