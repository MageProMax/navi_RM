#ifndef __NAV_PLAN_CONFIG_HPP__
#define __NAV_PLAN_CONFIG_HPP__
// Target coordinates are configured through waypoints.* ROS parameters.
// This legacy header keeps topic declarations for existing integrations.
class NavPlanConfig {
public:
    static const char* TOPIC_GAME_STATE;
    static const char* TOPIC_ROBOT_STATE;
    static const char* TOPIC_TARGET_POSE;
};
#endif
