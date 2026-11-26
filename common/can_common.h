#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include <stdint.h>

// CAN arb is 11 bits, 3 MSB for type, 8 LSB for node
// NOTE: using classic can, so only 8 byte payload max

// MSG TYPES (3 MSB, note that lower values = higher priority)
#define CAN_TYPE_ESTOP (0x0u << 8)
#define CAN_TYPE_COMMAND (0x1u << 8)
#define CAN_TYPE_FEEDBACK (0x2u << 8)
#define CAN_TYPE_SENSOR (0x3u << 8)
#define CAN_TYPE_HEARTBEAT (0x7u << 8)

// NODE IDS (8 LSB)
#define LOW_VOLTAGE_BOARD 0x01
#define THRUSTER_CONTROLLER 0x02
#define BMS 0x03
#define LED_INDICATOR 0x04
#define CANNON_PUMP 0x05

// HELPERS
#define MAKE_CAN_ID(type, node) ((type) | (node))
#define GET_CAN_NODE(id) ((id) & 0xFF)
#define GET_CAN_MSG_TYPE(id) ((id) & 0x700)

// COMMON MSGS
#define CAN_ID_ESTOP 0x000
#define CAN_ID_ESTOP_RST 0x001
#define CAN_ID_HEARTBEAT(node) MAKE_CAN_ID(CAN_TYPE_HEARTBEAT, (node))
#define CAN_ID_THRUSTER_CMD MAKE_CAN_ID(CAN_TYPE_COMMAND, THRUSTER_CONTROLLER)
#define CAN_ID_FIRE_CMD MAKE_CAN_ID(CAN_TYPE_COMMAND, CANNON_PUMP)
#define CAN_ID_BATTERY_INFO MAKE_CAN_ID(CAN_TYPE_SENSOR, LOW_VOLTAGE_BOARD)

// ESTOP RST PAYLOAD
// prevent error from resetting estop
#define ESTOP_RST_0 0xA5
#define ESTOP_RST_1 0x5A

// HEARTBEAT
#define HEARTBEAT_PERIOD_MS 100
// 3 missed heartbeats -> assume node disconnected
#define HEARTBEAT_TIMEOUT_MS 350

// types
typedef struct __attribute__((packed)) {
  int8_t fl; // 1% * value,-100 = -100%, +100 = +100%
  int8_t fr;
  int8_t bl;
  int8_t br;
} thruster_cmds_t;

typedef struct __attribute__((packed)) {
  int16_t voltage_mV; // mV
  int16_t current_mA; // mA
} battery_info_t;

typedef struct __attribute__((packed)) {
  int8_t angle_mrad; // milliradians
  int8_t power;      // TBD
  int8_t target;     // 1 if cannon, -1 if pump
} fire_t;

#define FIRE_TARGET_NONE 0
#define FIRE_TARGET_CANNON 1
#define FIRE_TARGET_PUMP 2

// validate each payload fits byte requirement
#define CAN_PAYLOAD_CHECK(T)                                                   \
  _Static_assert(sizeof(T) <= 8, #T " too big for CAN")
CAN_PAYLOAD_CHECK(thruster_cmds_t);
CAN_PAYLOAD_CHECK(battery_info_t);
CAN_PAYLOAD_CHECK(fire_t);

#endif