#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stddef.h>
#include <stdint.h>

// CONTROL
#define THRUST_PCT_MAX 100
typedef struct {
  // percent of full thrust: -100 = full reverse, +100 = full
  int8_t fl;
  int8_t fr;
  int8_t bl;
  int8_t br;
} thruster_commands_t;

typedef enum : uint8_t {
  SOFT_ESTOP = 0x1,
  MANUAL = 0x2,
  AUTONOMOUS = 0x3,
  STATIONKEEPING = 0x4,
} boat_state_t;

typedef enum : uint8_t {
  REST = 0x1,
  FIRE_CANNON = 0x2,
  FIRE_PUMP = 0x3,
} launch_actions_t;

// SENSOR
// SCH16T-K01 (20-bit mode)
#pragma pack(push, 1)
typedef struct {
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;
  int32_t accel_x;
  int32_t accel_y;
  int32_t accel_z;
  uint16_t status;
} imu_data_t;
#pragma pack(pop)

// MMC5983MA (18-bit mode)
#pragma pack(push, 1)
typedef struct {
  int32_t mag_x;
  int32_t mag_y;
  int32_t mag_z;
  uint8_t status;
} mag_data_t;
#pragma pack(pop)

// scale factors
#define IMU_GYRO_LSB_PER_DPS 1600.0f
#define IMU_ACCEL_LSB_PER_MS2 3200.0f
#define IMU_TEMP_LSB_PER_C 100.0f
#define MAG_COUNTS_PER_GAUSS 16384.0f
#define MAG_NULL_OFFSET 131072

#endif // COMMON_TYPES_H