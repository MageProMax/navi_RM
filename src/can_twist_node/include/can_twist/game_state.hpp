#ifndef __GAME_STATE_HPP__
#define __GAME_STATE_HPP__

#include <cstdint>
#include <cstring>

/**
 * @brief Game progress stage enumeration
 * Stage progression: 0->1->2->3->4->5
 */
enum class GameProgress : uint8_t {
    IDLE = 0,              // Not started
    PREPARATION = 1,       // Preparation phase
    SELF_CHECK = 2,        // 15s self-check
    COUNTDOWN = 3,         // 5s countdown
    PLAYING = 4,           // Match in progress
    SETTLEMENT = 5         // Match settlement
};

/**
 * @brief HP deduction reason
 */
enum class HPDeductionReason : uint8_t {
    ARMOR_ATTACKED = 0,    // Armor module hit by projectile
    OFFLINE = 1,           // Armor/capacitor module offline
    COLLISION = 5          // Armor module collision
};

/**
 * @brief Game arena state (CAN ID: 0x153)
 * Received from referee system, little-endian
 */
struct GamePara {
    uint8_t game_progress : 4;      // bit 0-3: current game stage
    uint8_t HP_deduction_reason : 4; // bit 4-7: HP deduction type
    uint16_t stage_remain_time;     // remaining time in current stage (seconds)
    uint8_t level;                  // last penalty level
    uint8_t offending_robot_id;     // offending robot ID
    uint8_t recovery_occupy : 1;    // bit 0: recovery point occupied (1=yes)
    uint8_t mid_occupy : 2;         // bit 1-2: center buff (0=free, 1=ours, 2=enemy, 3=both)
    uint8_t recovery_rfid : 1;      // bit 3: recovery RFID detected
    uint8_t mid_rfid : 1;           // bit 4: center RFID detected
    uint8_t power_management_shooter_output : 1; // bit 5: shooter power (1=24V on)
    uint8_t recovery_buff;          // recovery buff percentage (10 = 10%)
    uint8_t shooter_remain;
    /**
     * @brief Parse 0x153 CAN frame (8 bytes, little-endian)
     * @param data CAN frame payload
     */
    void parse_0x153(const uint8_t* data) {
        game_progress = data[0] & 0x0F;
        HP_deduction_reason = (data[0] >> 4) & 0x0F;
        stage_remain_time = static_cast<uint16_t>(data[1]) | 
                           (static_cast<uint16_t>(data[2]) << 8);
        level = data[3];
        offending_robot_id = data[4];
        recovery_occupy = (data[5] >> 0) & 0x01;
        mid_occupy = (data[5] >> 1) & 0x03;
        recovery_rfid = (data[5] >> 3) & 0x01;
        mid_rfid = (data[5] >> 4) & 0x01;
        power_management_shooter_output = (data[5] >> 5) & 0x01;
        recovery_buff = data[6];
        shooter_remain = data[7];
    }

    /**
     * @brief Get game progress enum
     */
    GameProgress get_game_progress() const {
        return static_cast<GameProgress>(game_progress);
    }

    /**
     * @brief Get HP deduction reason enum
     */
    HPDeductionReason get_hp_deduction_reason() const {
        return static_cast<HPDeductionReason>(HP_deduction_reason);
    }
};

/**
 * @brief Robot own state (CAN ID: 0x155)
 * Received from referee system, little-endian
 */
struct OwnPara {
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint8_t HP_deduction_reason : 4;     // from 0x153, cached here
    uint8_t recovery_rfid : 1;           // from 0x153
    uint8_t mid_rfid : 1;                // from 0x153
    uint8_t power_management_shooter_output : 1; // from 0x153
    uint8_t recovery_buff;               // from 0x153
    uint8_t shooter_remain;              // from 0x153
    /**
     * @brief Parse 0x155 CAN frame (8 bytes, little-endian)
     * @param data CAN frame payload
     */
    void parse_0x155(const uint8_t* data) {
        current_HP = static_cast<uint16_t>(data[0]) | 
                    (static_cast<uint16_t>(data[1]) << 8);
        maximum_HP = static_cast<uint16_t>(data[2]) | 
                    (static_cast<uint16_t>(data[3]) << 8);
    }

    /**
     * @brief Calculate health percentage
     * @return current_HP / maximum_HP (0.0 to 1.0)
     */
    float get_health_ratio() const {
        if (maximum_HP == 0) return 1.0f;
        return static_cast<float>(current_HP) / static_cast<float>(maximum_HP);
    }
};

/**
 * @brief Allied robots state (CAN ID: 0x155 continuation or separate)
 * Used for tactical awareness
 */
struct AllyPara {
    uint16_t ally_1_robot_HP;      // Hero robot HP (0 if offline)
    uint16_t ally_3_robot_HP;      // Infantry robot HP (0 if offline)
    uint8_t level;                 // last penalty level
    uint8_t offending_robot_id;    // offending robot ID

    /**
     * @brief Parse ally data from 0x155 extended payload
     * @param data CAN frame payload
     */
    void parse_0x155_extended(const uint8_t* data) {
        ally_1_robot_HP = static_cast<uint16_t>(data[4]) | 
                         (static_cast<uint16_t>(data[5]) << 8);
        ally_3_robot_HP = static_cast<uint16_t>(data[6]) | 
                         (static_cast<uint16_t>(data[7]) << 8);
    }
};

#endif // __GAME_STATE_HPP__
