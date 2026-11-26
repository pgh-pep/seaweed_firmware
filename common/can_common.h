#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include <stdint.h>

// CAN arb is 11 bits, 3 MSB for type, 8 LSB for node

// MSG TYPES (3 MSB, note that lower values = higher priority)
#define CAN_TYPE_ESTOP (0x0u << 8)
#define CAN_TYPE_COMMAND (0x1u << 8)
#define CAN_TYPE_FEEDBACK (0x2u << 8)
#define CAN_TYPE_SENSOR (0x3u << 8)
#define CAN_TYPE_HEARTBEAT (0x7u << 8)

// NODE IDS (8 LSB)
#define LOW_VOLTAGE_BOARD 0x00
#define THRUSTER_CONTROLLER 0x01
#define BMS 0x02
#define LED_INDICATOR 0x03
#define CANNON_PUMP 0x04

// HELPERS
#define MAKE_CAN_ID(type, node) ((type) | (node))
#define GET_CAN_NODE(id) ((id) & 0xFFu)
#define GET_CAN_MSG_TYPE(id) ((id) & 0x700u)

// COMMON MSGS
#define CAN_ID_ESTOP MAKE_CAN_ID(CAN_TYPE_ESTOP, LOW_VOLTAGE_BOARD)
#define CAN_ID_HEARTBEAT(node) MAKE_CAN_ID(CAN_TYPE_HEARTBEAT, (node))
#define CAN_ID_THRUSTER_CMD MAKE_CAN_ID(CAN_TYPE_COMMAND, THRUSTER_CONTROLLER)
#define CAN_ID_FIRE_CMD MAKE_CAN_ID(CAN_TYPE_COMMAND, CANNON_PUMP)
#define CAN_ID_BATTERY_INFO MAKE_CAN_ID(CAN_TYPE_SENSOR, LOW_VOLTAGE_BOARD)
#define CAN_ID_BOAT_STATE MAKE_CAN_ID(CAN_TYPE_COMMAND, LED_INDICATOR)

// HEARTBEAT
#define HEARTBEAT_PERIOD_MS 100
#define HEARTBEAT_TIMEOUT_MS 350 // 3 missed heartbeats -> assume node disconn

// standardized payloads (8-bytes max)
typedef struct __attribute__((packed)) {
  int8_t fl; // 1% * value, -100 = -100%, +100 = +100%
  int8_t fr;
  int8_t bl;
  int8_t br;
} thruster_cmds_t;

typedef struct __attribute__((packed)) {
  uint16_t voltage_mV; // mV
  int16_t current_mA;  // mA
} battery_info_t;

typedef enum : uint8_t {
  FIRE_TARGET_NONE = 0,
  FIRE_TARGET_CANNON,
  FIRE_TARGET_PUMP
} fire_target_t;

typedef struct __attribute__((packed)) {
  uint8_t angle_mrad;   // milliradians
  uint8_t power;        // TBD
  fire_target_t target; // none vs cannon vs pump
} fire_t;

// validate each payload fits byte requirement
#define CAN_PAYLOAD_CHECK(T)                                                   \
  _Static_assert(sizeof(T) <= 8, #T " too big for CAN")
CAN_PAYLOAD_CHECK(thruster_cmds_t);
CAN_PAYLOAD_CHECK(battery_info_t);
CAN_PAYLOAD_CHECK(fire_t);

#endif