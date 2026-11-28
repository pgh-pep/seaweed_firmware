#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include "common_types.h"

// CAN arb is 11 bits, lower id = higher priority
// NOTE: classic can, so 8B max payload

// MESSAGE IDS w/ PAYLOADS
#define CAN_ID_ESTOP 0x000u        // N/A
#define CAN_ID_ESTOP_RST 0x001u    // confirmation
#define CAN_ID_THRUSTER_CMD 0x100u // thruster_commands_t
#define CAN_ID_LAUNCHER_CMD 0x110u // launcher_cmd_t
#define CAN_ID_BOAT_STATE 0x120u   // boat_state_msg_t
#define CAN_ID_BATTERY_INFO 0x300u // battery_info_t

// TIMING
#define CAN_THRUSTER_PERIOD_MS 20u
#define CAN_LAUNCHER_PERIOD_MS 50u
#define CAN_STATE_PERIOD_MS 100u
#define CAN_ESTOP_REPEAT_MS 200u
#define CAN_CMD_TIMEOUT_MS 500u

// timeout
_Static_assert(CAN_CMD_TIMEOUT_MS >= 5u * CAN_THRUSTER_PERIOD_MS,
               "thruster timeout too tight");
_Static_assert(CAN_CMD_TIMEOUT_MS >= 5u * CAN_LAUNCHER_PERIOD_MS,
               "launcher timeout too tight");

// PAYLOADS:
// ESTOP
#define ESTOP_RST_0 0xA5u
#define ESTOP_RST_1 0x5Au

// thurster command = thruster_commands_t (common_types)

// LAUNCHER COMMAND
// firing edge trigg based on shot_id
// new shot_id -> fire action
typedef struct __attribute__((packed)) {
  uint16_t angle_deg; // launcher angle (degrees)
  uint8_t power;      // TBD
  uint8_t action;     // launch_actions_t
  uint8_t shot_id;    // increments once per fire request
} launcher_cmd_t;

// BOAT STATE
typedef struct __attribute__((packed)) {
  uint8_t state; // boat_state_t
} boat_state_msg_t;

// BATTERY INFO
typedef struct __attribute__((packed)) {
  uint16_t voltage_mV;
  int32_t current_mA;
} battery_info_t;

// validate each payload fits in a classic CAN frame
#define CAN_PAYLOAD_CHECK(T) _Static_assert(sizeof(T) <= 8, #T " too big")
_Static_assert(sizeof(thruster_commands_t) == 4, "thruster cmd layout changed");
CAN_PAYLOAD_CHECK(launcher_cmd_t);
CAN_PAYLOAD_CHECK(boat_state_msg_t);
CAN_PAYLOAD_CHECK(battery_info_t);

#endif // CAN_COMMON_H