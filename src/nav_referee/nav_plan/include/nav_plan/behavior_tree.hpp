#ifndef __BEHAVIOR_TREE_HPP__
#define __BEHAVIOR_TREE_HPP__

#include <cstdint>
#include <string>
#include <memory>
#include <functional>
#include <vector>

/**
 * @brief Behavior Tree state machine for robot decision making
 * 
 * State transitions:
 * - STARTUP -> RECOVERY (initial spawn point)
 * - RECOVERY -> OCCUPY (when healthy + powered, game starts)
 * - OCCUPY -> RECOVERY (various damage/power conditions)
 * - RECOVERY -> DORMANT (on settlement or permanent damage)
 * - DORMANT -> (no transitions, terminal state)
 */

enum class BTState : uint8_t {
    STARTUP = 0,     // Initial state, not yet started
    RECOVERY = 1,    // At recovery point, healing
    OCCUPY = 2,      // Moving to/occupying objective
    DORMANT = 3      // Match ended or permanent damage
};

enum class TargetType : uint8_t {
    RECOVERY = 0,
    OCCUPY = 1,
    ATTACK = 2,
    DEFEND = 3
};

/**
 * @brief Navigation waypoint with coordinates and metadata
 */
struct Waypoint {
    float x;
    float y;
    float z;
    TargetType type;
    std::string name;

    Waypoint(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f, 
             TargetType type_ = TargetType::RECOVERY, const std::string& name_ = "")
        : x(x_), y(y_), z(z_), type(type_), name(name_) {}
};

/**
 * @brief Behavior Tree decision node context
 * Encapsulates all state needed for decision making
 */
struct BTContext {
    // Game state inputs (0x153)
    uint8_t game_progress;                      // 0-5 (see GameProgress enum)
    uint16_t stage_remain_time;
    
    // Robot state inputs (0x155)
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint8_t HP_deduction_reason;                // 0=armor_attack, 1=offline, 5=collision
    uint8_t power_management_shooter_output;    // 0=off, 1=on
    
    // Ally state inputs (0x155 extended)
    uint16_t ally_1_robot_HP;
    uint16_t ally_3_robot_HP;
    
    // Thresholds
    float health_threshold;
    
    // Waypoints
    std::vector<Waypoint> waypoints;
    
    // Current state
    BTState current_state;
    Waypoint target_waypoint;
    
    /**
     * @brief Calculate health ratio (0.0 to 1.0)
     * @return current_HP / maximum_HP
     */
    float get_health_ratio() const {
        if (maximum_HP == 0) return 0.0f;
        return static_cast<float>(current_HP) / static_cast<float>(maximum_HP);
    }
    
    /**
     * @brief Calculate if robot is critically damaged
     */
    bool is_critically_damaged() const {
        if (maximum_HP == 0) return false;
        return get_health_ratio() <= health_threshold;
    }
    
    /**
     * @brief Check if robot is fully healed
     */
    bool is_fully_healed() const {
        return current_HP >= maximum_HP;
    }
    
    /**
     * @brief Check if match is in progress
     */
    bool is_match_active() const {
        return game_progress == 4; // PLAYING state
    }
    
    /**
     * @brief Check if match has ended
     */
    bool is_match_ended() const {
        return game_progress == 5; // SETTLEMENT state
    }
    
    /**
     * @brief Check for permanent offline damage
     */
    bool has_permanent_damage() const {
        return (HP_deduction_reason == 1); // Module offline
    }
};

/**
 * @brief Behavior Tree node interface
 * Each node represents a decision/action in the tree
 */
class BTNode {
public:
    enum class Status {
        SUCCESS,
        FAILURE,
        RUNNING
    };
    
    virtual ~BTNode() = default;
    
    /**
     * @brief Execute the node with given context
     * @return Status of execution
     */
    virtual Status tick(BTContext& context) = 0;
    
    /**
     * @brief Get human-readable node name
     */
    virtual std::string get_name() const = 0;
};

/**
 * @brief Check if match just started (transition 3->4)
 */
class MatchStartCheckNode : public BTNode {
private:
    uint8_t last_progress_ = 0;
    
public:
    Status tick(BTContext& context) override {
        if (last_progress_ == 3 && context.game_progress == 4) {
            last_progress_ = context.game_progress;
            return Status::SUCCESS;
        }
        last_progress_ = context.game_progress;
        return Status::FAILURE;
    }
    
    std::string get_name() const override { return "MatchStartCheck"; }
};

/**
 * @brief Condition: Is robot healthy and powered?
 * Requirements:
 *   - Power output is ON (shooter power available)
 *   - Health is sufficient (at least 80% of maximum)
 */
class IsHealthyAndPoweredNode : public BTNode {
public:
    Status tick(BTContext& context) override {
        // Check if power is ON
        /*if (context.power_management_shooter_output != 1) {
            return Status::FAILURE;
        }*/
        
        // Check if health is sufficient (> 80% of maximum)
        // More flexible than requiring full health
        float health_ratio = context.get_health_ratio();
        const float HEALTH_OCCUPY_THRESHOLD = 0.8f;  // 80% threshold for occupation
        
        if (health_ratio <= HEALTH_OCCUPY_THRESHOLD) {
            return Status::FAILURE;
        }
        
        return Status::SUCCESS;
    }
    
    std::string get_name() const override { return "IsHealthyAndPowered"; }
};

/**
 * @brief Condition: Should return to recovery?
 * Returns SUCCESS if any recovery condition is met:
 *   1. Power is OFF -> charge
 *   2. Health critically low -> heal
 *   3. Match ended -> stop at recovery
 *   4. Module offline (permanent damage) -> stay at recovery
 */
class ShouldReturnToRecoveryNode : public BTNode {
public:
    Status tick(BTContext& context) override {
        // Condition 1: Power management off -> return to charge
        /*if (context.power_management_shooter_output == 0) {
            return Status::SUCCESS;
        }*/
        
        // Condition 2: Critical damage (HP < 25%) -> return to heal
        if (context.is_critically_damaged()) {
            return Status::SUCCESS;
        }
        
        // Condition 3: Match ended (game_progress == 5) -> return to settle
        if (context.is_match_ended()) {
            return Status::SUCCESS;
        }
        
        // Condition 4: Permanent module offline -> return (terminal state)
        if (context.has_permanent_damage()) {
            return Status::SUCCESS;
        }
        
        return Status::FAILURE;
    }
    
    std::string get_name() const override { return "ShouldReturnToRecovery"; }
};

/**
 * @brief Action: Set target to recovery point
 */
class SetRecoveryTargetNode : public BTNode {
public:
    Status tick(BTContext& context) override {
        for (const auto& wp : context.waypoints) {
            if (wp.type == TargetType::RECOVERY) {
                context.target_waypoint = wp;
                return Status::SUCCESS;
            }
        }
        return Status::FAILURE;
    }
    
    std::string get_name() const override { return "SetRecoveryTarget"; }
};

/**
 * @brief Action: Set target to occupation point
 */
class SetOccupyTargetNode : public BTNode {
public:
    Status tick(BTContext& context) override {
        for (const auto& wp : context.waypoints) {
            if (wp.type == TargetType::OCCUPY) {
                context.target_waypoint = wp;
                return Status::SUCCESS;
            }
        }
        return Status::FAILURE;
    }
    
    std::string get_name() const override { return "SetOccupyTarget"; }
};

/**
 * @brief Composite selector node (OR logic)
 * Returns SUCCESS if any child returns SUCCESS
 */
class SelectorNode : public BTNode {
private:
    std::vector<std::shared_ptr<BTNode>> children_;
    
public:
    void add_child(std::shared_ptr<BTNode> child) {
        children_.push_back(child);
    }
    
    Status tick(BTContext& context) override {
        for (auto& child : children_) {
            if (child->tick(context) == Status::SUCCESS) {
                return Status::SUCCESS;
            }
        }
        return Status::FAILURE;
    }
    
    std::string get_name() const override { return "Selector"; }
};

/**
 * @brief Composite sequence node (AND logic)
 * Returns SUCCESS if all children return SUCCESS
 */
class SequenceNode : public BTNode {
private:
    std::vector<std::shared_ptr<BTNode>> children_;
    
public:
    void add_child(std::shared_ptr<BTNode> child) {
        children_.push_back(child);
    }
    
    Status tick(BTContext& context) override {
        for (auto& child : children_) {
            if (child->tick(context) != Status::SUCCESS) {
                return Status::FAILURE;
            }
        }
        return Status::SUCCESS;
    }
    
    std::string get_name() const override { return "Sequence"; }
};

/**
 * @brief Root behavior tree manager
 */
class BehaviorTree {
private:
    std::shared_ptr<BTNode> root_;
    BTContext context_;
    
public:
    BehaviorTree() : root_(nullptr) {
        context_.current_state = BTState::STARTUP;
    }
    
    void set_root(std::shared_ptr<BTNode> root) {
        root_ = root;
    }
    
    BTContext& get_context() {
        return context_;
    }
    
    /**
     * @brief Execute one tree tick
     */
    void tick() {
        if (!root_) return;
        root_->tick(context_);
    }
    
    /**
     * @brief Update context from external game state
     */
    void update_context(uint8_t game_progress, uint16_t current_HP, uint16_t maximum_HP,
                       uint8_t HP_reason, uint8_t power_output) {
        context_.game_progress = game_progress;
        context_.current_HP = current_HP;
        context_.maximum_HP = maximum_HP;
        context_.HP_deduction_reason = HP_reason;
        context_.power_management_shooter_output = power_output;
    }
};

#endif // __BEHAVIOR_TREE_HPP__
