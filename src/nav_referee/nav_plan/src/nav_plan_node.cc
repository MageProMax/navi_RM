#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "rm_interfaces/msg/goal.hpp"
#include "rm_interfaces/srv/configure_patrol.hpp"
#include <map>
#include "rm_interfaces/msg/game_para.hpp"
#include "rm_interfaces/msg/own_para.hpp"
#include "rm_interfaces/msg/ally_para.hpp"
#include "nav_plan/behavior_tree.hpp"
#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <stdexcept>

/**
 * @brief Navigation planning node
 * 
 * Subscribes to game state and robot state, executes behavior tree,
 * and publishes target pose to path planning system.
 */
class NavPlanNode : public rclcpp::Node {
public:
    NavPlanNode() : rclcpp::Node("nav_plan_node") {
        // Initialize behavior tree with waypoints and health threshold
        behavior_tree_ = std::make_shared<BehaviorTree>();
        build_behavior_tree();
        
        // Get context and set initial parameters
        auto& context = behavior_tree_->get_context();
        // Coordinates are supplied by the selected map's ROS parameter file.
        for (const auto& entry : std::vector<std::pair<std::string, TargetType>>{
                 {"recovery", TargetType::RECOVERY}, {"occupy", TargetType::OCCUPY},
                 {"attack", TargetType::ATTACK}, {"defend", TargetType::DEFEND}}) {
            const auto xyz = declare_parameter<std::vector<double>>(
                "waypoints." + entry.first, std::vector<double>{});
            if (xyz.size() != 3 || !std::all_of(xyz.begin(), xyz.end(),
                    [](double v) { return std::isfinite(v); }))
                throw std::invalid_argument("waypoints." + entry.first + " must contain finite [x,y,z]");
            context.waypoints.emplace_back(xyz[0], xyz[1], xyz[2], entry.second, entry.first);
        }
        context.health_threshold = declare_parameter<double>("health_threshold", 0.25);
        if (!std::isfinite(context.health_threshold) || context.health_threshold <= 0 ||
            context.health_threshold > 1)
            throw std::invalid_argument("health_threshold must be in (0,1]");
        
        // Set initial target to recovery point
        for (const auto& wp : context.waypoints) {
            if (wp.type == TargetType::RECOVERY) {
                context.target_waypoint = wp;
                break;
            }
        }
        
        // Subscribe to game state (0x153)
        game_para_sub_ = this->create_subscription<rm_interfaces::msg::GamePara>(
            "game_para", 10,
            std::bind(&NavPlanNode::game_para_callback, this, std::placeholders::_1));
        
        // Subscribe to own robot state (0x155)
        own_para_sub_ = this->create_subscription<rm_interfaces::msg::OwnPara>(
            "own_para", 10,
            std::bind(&NavPlanNode::own_para_callback, this, std::placeholders::_1));
        
        // Subscribe to ally state (optional)
        ally_para_sub_ = this->create_subscription<rm_interfaces::msg::AllyPara>(
            "ally_para", 10,
            std::bind(&NavPlanNode::ally_para_callback, this, std::placeholders::_1));
        
        // Publisher for target goal
        target_goal_pub_ = this->create_publisher<rm_interfaces::msg::Goal>(
            "goal_pose", 10);
        
        // State transition logger
        state_logger_pub_ = this->create_publisher<std_msgs::msg::String>(
            "nav_plan_state", 10);
        
        // Timer for behavior tree execution (100 Hz)
        tree_timer_ = rclcpp::create_timer(this, this->get_clock(),
            rclcpp::Duration::from_seconds(0.01),
            std::bind(&NavPlanNode::tree_tick, this));
        
        // Timer for target pose publishing (0.2 Hz)
        target_pose_timer_ = rclcpp::create_timer(this, this->get_clock(),
            rclcpp::Duration::from_seconds(5.0),  // 0.2 Hz in the selected clock
            std::bind(&NavPlanNode::publish_target_pose_timer, this));
        
        configure_srv_ = create_service<rm_interfaces::srv::ConfigurePatrol>("/navigation/decision/configure",
          [this](rm_interfaces::srv::ConfigurePatrol::Request::SharedPtr req,
                 rm_interfaces::srv::ConfigurePatrol::Response::SharedPtr res) {
            if (req->mode!="formal" || req->frame_id!="map" || req->point_names.size()!=4 ||
                req->points.size()!=4 || !std::isfinite(req->health_threshold) ||
                req->health_threshold<=0 || req->health_threshold>1) {
              res->message="正式模式需要 map 下的四类目标和有效血量阈值";return;
            }
            const std::map<std::string,TargetType> types={{"recovery",TargetType::RECOVERY},
              {"occupy",TargetType::OCCUPY},{"attack",TargetType::ATTACK},{"defend",TargetType::DEFEND}};
            std::vector<Waypoint> points;
            std::map<std::string,bool> seen;
            for(size_t i=0;i<4;++i) {
              const auto& name=req->point_names[i];const auto& p=req->points[i];
              if(!types.count(name) || seen[name] || !std::isfinite(p.goal_x) ||
                 !std::isfinite(p.goal_y) || !std::isfinite(p.goal_z)) {
                res->message="目标类型重复或坐标无效";return;
              }
              seen[name]=true;points.emplace_back(p.goal_x,p.goal_y,p.goal_z,types.at(name),name);
            }
            auto& ctx=behavior_tree_->get_context();ctx.waypoints=points;
            ctx.health_threshold=req->health_threshold;
            for(const auto& p:points) if(p.type==ctx.target_waypoint.type) ctx.target_waypoint=p;
            res->success=true;res->message="正式决策目标已更新";
          });
        RCLCPP_INFO(this->get_logger(), "Nav Plan Node Started");
        RCLCPP_INFO(this->get_logger(), "Health threshold: %.2f", context.health_threshold);
        RCLCPP_INFO(this->get_logger(), "Initial target: %s (%.2f, %.2f)", 
                   context.target_waypoint.name.c_str(),
                   context.target_waypoint.x, context.target_waypoint.y);
    }

private:
    rclcpp::Service<rm_interfaces::srv::ConfigurePatrol>::SharedPtr configure_srv_;
    /**
     * @brief Build the behavior tree structure
     * 
     * Tree logic:
     * 1. If match ended or permanent damage -> stay in recovery
     * 2. Else if should return to recovery -> go to recovery
     * 3. Else if match started and healthy/powered -> go to occupy
     * 4. Else -> stay at current target
     */
    void build_behavior_tree() {
        // Match start check node
        auto match_start_check = std::make_shared<MatchStartCheckNode>();
        
        // Condition nodes
        auto is_healthy_and_powered = std::make_shared<IsHealthyAndPoweredNode>();
        auto should_return = std::make_shared<ShouldReturnToRecoveryNode>();
        
        // Action nodes
        auto set_recovery_target = std::make_shared<SetRecoveryTargetNode>();
        auto set_occupy_target = std::make_shared<SetOccupyTargetNode>();
        
        // Build tree structure
        // Priority: Recovery > Occupy > Stay
        auto root = std::make_shared<SelectorNode>();
        
        // Branch 1: Should return to recovery?
        auto recovery_sequence = std::make_shared<SequenceNode>();
        recovery_sequence->add_child(should_return);
        recovery_sequence->add_child(set_recovery_target);
        root->add_child(recovery_sequence);
        
        // Branch 2: Match just started and robot healthy?
        auto occupy_sequence = std::make_shared<SequenceNode>();
        occupy_sequence->add_child(match_start_check);
        occupy_sequence->add_child(is_healthy_and_powered);
        occupy_sequence->add_child(set_occupy_target);
        root->add_child(occupy_sequence);
        
        // Branch 3: Robot recovered, now go occupy
        auto recovery_complete_sequence = std::make_shared<SequenceNode>();
        auto is_fully_healed = std::make_shared<IsFullyHealedNode>();
        auto is_powered = std::make_shared<IsPoweredNode>();
        recovery_complete_sequence->add_child(is_fully_healed);
        recovery_complete_sequence->add_child(is_powered);
        recovery_complete_sequence->add_child(set_occupy_target);
        root->add_child(recovery_complete_sequence);
        
        behavior_tree_->set_root(root);
    }
    
    /**
     * @brief Condition: Is robot fully healed?
     */
    class IsFullyHealedNode : public BTNode {
    public:
        Status tick(BTContext& context) override {
            return context.is_fully_healed() ? Status::SUCCESS : Status::FAILURE;
        }
        std::string get_name() const override { return "IsFullyHealed"; }
    };
    
    /**
     * @brief Condition: Is power output on?
     */
    class IsPoweredNode : public BTNode {
    public:
        Status tick(BTContext& /* context */) override {
            return Status::SUCCESS; //context.power_management_shooter_output == 1 ? Status::SUCCESS : Status::FAILURE;
        }
        std::string get_name() const override { return "IsPowered"; }
    };

    /**
     * @brief Execute one tick of behavior tree
     */
    void tree_tick() {
        // Execute behavior tree decision logic
        behavior_tree_->tick();
        
        auto& context = behavior_tree_->get_context();
        
        // Log current state (throttled to avoid spam)
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "BT Decision: progress=%d, HP=%d/%d (%.1f%%), power=%d, target=%s",
            context.game_progress, 
            context.current_HP, context.maximum_HP, context.get_health_ratio() * 100,
            context.power_management_shooter_output, 
            context.target_waypoint.name.c_str());
        
        // Update state machine based on conditions
        update_state(context);
    }

    /**
     * @brief Update internal state based on behavior tree context
     */
    void update_state(BTContext& context) {
        BTState new_state = context.current_state;
        
        if (context.is_match_ended() || 
            (context.has_permanent_damage() && context.HP_deduction_reason == 1)) {
            new_state = BTState::DORMANT;
        } else if (context.is_critically_damaged() /*|| 
                   context.power_management_shooter_output == 0*/) {
            new_state = BTState::RECOVERY;
        } else if (context.is_match_active() && !context.is_critically_damaged() /*&& 
                   context.power_management_shooter_output == 1*/) {
            new_state = BTState::OCCUPY;
        }
        
        if (new_state != context.current_state) {
            context.current_state = new_state;
            log_state_transition(new_state);
        }
    }

    /**
     * @brief Publish current target pose
     * 
     * Only publish target position when match is active (game_progress >= 4)
     * When game_progress is 0-3 (before match starts), don't publish target data
     */
    void publish_target_pose(const BTContext& context) {
        // Before the match, no target means no motion; (0,0) is a real goal.
        if (context.game_progress < 4) return;
        auto goal_msg = std::make_shared<rm_interfaces::msg::Goal>();
        
        goal_msg->goal_frame_id = "map";
        
        if (context.game_progress >= 4) {
            goal_msg->goal_x = context.target_waypoint.x;
            goal_msg->goal_y = context.target_waypoint.y;
            goal_msg->goal_z = context.target_waypoint.z;
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Publishing target: %s (%.2f, %.2f)", 
                context.target_waypoint.name.c_str(),
                context.target_waypoint.x, context.target_waypoint.y);
        }

        target_goal_pub_->publish(*goal_msg);
    }

    /**
     * @brief Log state transitions for debugging
     */
    void log_state_transition(BTState new_state) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        
        switch (new_state) {
            case BTState::STARTUP:
                msg->data = "STATE: STARTUP";
                RCLCPP_INFO(this->get_logger(), "Transition: STARTUP");
                break;
            case BTState::RECOVERY:
                msg->data = "STATE: RECOVERY - Returning to recovery point";
                RCLCPP_WARN(this->get_logger(), "Transition: RECOVERY");
                break;
            case BTState::OCCUPY:
                msg->data = "STATE: OCCUPY - Moving to objective";
                RCLCPP_INFO(this->get_logger(), "Transition: OCCUPY");
                break;
            case BTState::DORMANT:
                msg->data = "STATE: DORMANT - Match ended or permanent damage";
                RCLCPP_ERROR(this->get_logger(), "Transition: DORMANT");
                break;
        }
        
        state_logger_pub_->publish(*msg);
    }

    /**
     * @brief Timer callback for publishing target pose (0.2 Hz)
     */
    void publish_target_pose_timer() {
        auto& context = behavior_tree_->get_context();
        publish_target_pose(context);
    }

    /**
     * @brief Process GamePara message from can_twist_node (CAN 0x153)
     */
    void game_para_callback(const rm_interfaces::msg::GamePara::SharedPtr msg) {
        auto& context = behavior_tree_->get_context();
        context.game_progress = msg->game_progress;
        context.stage_remain_time = msg->stage_remain_time;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "GamePara: progress=%d, remain_time=%d", 
            msg->game_progress, msg->stage_remain_time);
    }

    /**
     * @brief Process OwnPara message from can_twist_node (CAN 0x155)
     */
    void own_para_callback(const rm_interfaces::msg::OwnPara::SharedPtr msg) {
        auto& context = behavior_tree_->get_context();
        context.current_HP = msg->current_hp;
        context.maximum_HP = msg->maximum_hp;
        context.HP_deduction_reason = msg->hp_deduction_reason;
        context.power_management_shooter_output = msg->power_management_shooter_output;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "OwnPara: HP=%d/%d (%.1f%%), power=%d", 
            msg->current_hp, msg->maximum_hp,
            (float)msg->current_hp / msg->maximum_hp * 100,
            msg->power_management_shooter_output);
    }

    /**
     * @brief Process AllyPara message from can_twist_node (CAN 0x155 extended)
     */
    void ally_para_callback(const rm_interfaces::msg::AllyPara::SharedPtr msg) {
        auto& context = behavior_tree_->get_context();
        context.ally_1_robot_HP = msg->ally_1_robot_hp;
        context.ally_3_robot_HP = msg->ally_3_robot_hp;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "AllyPara: ally_1_HP=%d, ally_3_HP=%d", 
            msg->ally_1_robot_hp, msg->ally_3_robot_hp);
    }

    // Member variables
    std::shared_ptr<BehaviorTree> behavior_tree_;
    
    rclcpp::Subscription<rm_interfaces::msg::GamePara>::SharedPtr game_para_sub_;
    rclcpp::Subscription<rm_interfaces::msg::OwnPara>::SharedPtr own_para_sub_;
    rclcpp::Subscription<rm_interfaces::msg::AllyPara>::SharedPtr ally_para_sub_;
    
    rclcpp::Publisher<rm_interfaces::msg::Goal>::SharedPtr target_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_logger_pub_;
    
    rclcpp::TimerBase::SharedPtr tree_timer_;
    rclcpp::TimerBase::SharedPtr target_pose_timer_;  // Timer for target pose publishing (0.2 Hz)
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavPlanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
