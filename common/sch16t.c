#include "sch16t.h"
#include "main.h"
#include "spi.h"

#define IMU_SPI (&hspi1)
#define IMU_SPI_TIMEOUT_MS 10u

// REGISTERS
#define REG_RATE_X1 0x01
#define REG_RATE_Y1 0x02
#define REG_RATE_Z1 0x03
#define REG_ACC_X1 0x04
#define REG_ACC_Y1 0x05
#define REG_ACC_Z1 0x06
#define REG_STAT_SUM 0x14
#define REG_CTRL_MODE 0x35

#define MODE_EN_SENSOR (1u << 0)
#define MODE_EOI (1u << 1)

/* STAT_SUM, STAT_SUM_SAT, STAT_COM, STAT_RATE_COM, STAT_RATE_X/Y/Z,
 * STAT_ACC_X/Y/Z */
#define STAT_COUNT 10
static const uint8_t stat_regs[STAT_COUNT] = {0x14, 0x15, 0x16, 0x17, 0x18,
                                              0x19, 0x1A, 0x1B, 0x1C, 0x1D};

static const uint8_t data_regs[6] = {REG_RATE_X1, REG_RATE_Y1, REG_RATE_Z1,
                                     REG_ACC_X1,  REG_ACC_Y1,  REG_ACC_Z1};

// crc8 required by IMU
static uint8_t crc8(uint64_t frame) {
  uint8_t c = 0xFF;
  for (int i = 47; i >= 8; i--) {
    uint8_t in = (frame >> i) & 1u;
    uint8_t top = c >> 7;
    c = (uint8_t)(c << 1);
    if (top ^ in)
      c ^= 0x2F;
  }
  return c ^ 0xD2;
}

// MOSI: TA[9:0] | RW | 0 | FT=1 | AE[6:0]=0 | DATA[19:0] | CRC8
static uint64_t build_frame(uint8_t addr, bool write, uint32_t data20) {
  uint64_t f = ((uint64_t)addr << 38) | ((uint64_t)write << 37) | (1ULL << 35) |
               ((uint64_t)(data20 & 0xFFFFFu) << 8);
  return f | crc8(f);
}

static bool transfer_frame(uint64_t tx, uint64_t *rx) {
  uint8_t t[6], r[6];
  for (int i = 0; i < 6; i++)
    t[i] = (uint8_t)(tx >> (40 - 8 * i));

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(IMU_SPI, t, r, 6, IMU_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  if (st != HAL_OK)
    return false;

  uint64_t v = 0;
  for (int i = 0; i < 6; i++)
    v = (v << 8) | r[i];
  *rx = v;
  return true;
}

// fetch data
static bool read_regs(const uint8_t *addrs, uint8_t n, uint32_t *raw,
                      uint8_t *sbits) {
  uint64_t rx;
  for (uint8_t i = 0; i <= n; i++) {
    uint8_t a = (i < n) ? addrs[i] : addrs[n - 1];
    if (!transfer_frame(build_frame(a, false, 0), &rx))
      return false;
    if (i == 0)
      continue;

    if (crc8(rx) != (uint8_t)rx)
      return false;
    if (((rx >> 37) & 0xFFu) != addrs[i - 1])
      return false;
    raw[i - 1] = (uint32_t)((rx >> 8) & 0xFFFFFu);
    sbits[i - 1] = (uint8_t)((rx >> 33) & 0x3u);
  }
  return true;
}

static bool write_reg(uint8_t addr, uint32_t data) {
  uint64_t rx;
  return transfer_frame(build_frame(addr, true, data), &rx);
}

bool imu_init(void) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  //   hw reset
  HAL_GPIO_WritePin(IMU_EXTRESN_GPIO_Port, IMU_EXTRESN_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(IMU_EXTRESN_GPIO_Port, IMU_EXTRESN_Pin, GPIO_PIN_SET);
  HAL_Delay(35);

  // default for now
  if (!write_reg(REG_CTRL_MODE, MODE_EN_SENSOR))
    return false;
  HAL_Delay(250);

  // status regs
  uint32_t raw[STAT_COUNT];
  uint8_t sb[STAT_COUNT];
  if (!read_regs(stat_regs, STAT_COUNT, raw, sb))
    return false;

  if (!write_reg(REG_CTRL_MODE, MODE_EN_SENSOR | MODE_EOI))
    return false;
  HAL_Delay(3);

  // status reg second pass
  if (!read_regs(stat_regs, STAT_COUNT, raw, sb))
    return false;
  if (!read_regs(stat_regs, STAT_COUNT, raw, sb))
    return false;
  for (int i = 0; i < STAT_COUNT; i++) {
    if ((raw[i] >> 4) != 0xFFFFu)
      return false;
  }
  return true;
}

bool imu_read(imu_data_t *out) {
  uint32_t raw[6];
  uint8_t sb[6];
  if (!read_regs(data_regs, 6, raw, sb))
    return false;

  int32_t v[6];
  uint16_t status = 0;
  for (int i = 0; i < 6; i++) {
    v[i] = ((int32_t)(raw[i] << 12)) >> 12; // sign-extend 20b -> 32b
    status |= (uint16_t)(sb[i] << (2 * i));
  }

  out->gyro_x = v[0];
  out->gyro_y = v[1];
  out->gyro_z = v[2];
  out->accel_x = v[3];
  out->accel_y = v[4];
  out->accel_z = v[5];
  out->status = status;
  return true;
}