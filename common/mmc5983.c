#include "mmc5983.h"
#include "main.h"
#include "spi.h"

#define MAG_SPI (&hspi2)
#define MAG_SPI_TIMEOUT_MS 10u

// regs
#define REG_XOUT0 0x00
#define REG_STATUS 0x08
#define REG_CTRL0 0x09
#define REG_CTRL1 0x0A
#define REG_CTRL2 0x0B
#define REG_PRODUCT_ID 0x2F

#define PRODUCT_ID 0x30

// ctrl bits
#define CTRL0_SET (1u << 3)
#define CTRL0_AUTO_SR_EN (1u << 5)
#define CTRL1_BW_800HZ 0x03u // bw = 11, 0.5 ms per meas
#define CTRL1_SW_RST (1u << 7)
#define CTRL2_CMM_EN (1u << 3)
#define CTRL2_CM_FREQ_1000 0x07u       // 1000 Hz, only works w/ bw = 11
#define CTRL2_PRD_SET_1000 (0x6u << 4) // auto SET every 1000 meas
#define CTRL2_EN_PRD_SET (1u << 7)

// burst order: X0 X1 Y0 Y1 Z0 Z1 XYZ2 TOUT STATUS
#define BURST_LEN 9

static bool read_regs(uint8_t addr, uint8_t *dst, uint8_t n) {
  uint8_t t[1 + BURST_LEN] = {0};
  uint8_t r[1 + BURST_LEN];
  if (n > BURST_LEN)
    return false;
  t[0] = 0x80u | addr; // bit7 = 1 for read, bits 5:0 = addr

  // cs low, transfer, cs high
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef st =
      HAL_SPI_TransmitReceive(MAG_SPI, t, r, 1 + n, MAG_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  if (st != HAL_OK)
    return false;

  // skip first byte, junk while addr was sent
  for (uint8_t i = 0; i < n; i++)
    dst[i] = r[1 + i];
  return true;
}

static bool write_reg(uint8_t addr, uint8_t val) {
  uint8_t t[2] = {addr & 0x3Fu, val}; // bit7 = 0 for write

  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(MAG_SPI, t, 2, MAG_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  return st == HAL_OK;
}

// debug, watch in live expressions
volatile uint8_t mag_last_id = 0xEE;

bool mag_init(void) {
  uint8_t id = 0;
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(15); // wait for power on

  // soft reset
  if (!write_reg(REG_CTRL1, CTRL1_SW_RST))
    return false;
  HAL_Delay(15);

  // check product id
  bool ok = read_regs(REG_PRODUCT_ID, &id, 1);
  mag_last_id = id;
  if (!ok || id != PRODUCT_ID)
    return false;

  if (!read_regs(REG_PRODUCT_ID, &id, 1) || id != PRODUCT_ID)
    return false;

  // ctrl regs are write only, so write full value every time
  if (!write_reg(REG_CTRL1, CTRL1_BW_800HZ))
    return false;
  // initial SET pulse
  if (!write_reg(REG_CTRL0, CTRL0_SET))
    return false;
  HAL_Delay(1);
  // auto set/reset
  if (!write_reg(REG_CTRL0, CTRL0_AUTO_SR_EN))
    return false;
  // continuous mode at 1000 Hz, periodic SET every 1000 meas
  if (!write_reg(REG_CTRL2, CTRL2_EN_PRD_SET | CTRL2_PRD_SET_1000 |
                                CTRL2_CMM_EN | CTRL2_CM_FREQ_1000))
    return false;
  return true;
}

bool mag_read(mag_data_t *out) {
  uint8_t b[BURST_LEN];
  if (!read_regs(REG_XOUT0, b, BURST_LEN))
    return false;

  // 18 bit per axis: high byte, mid byte, low 2 bits from XYZ2
  int32_t x = ((int32_t)b[0] << 10) | ((int32_t)b[1] << 2) | (b[6] >> 6);
  int32_t y = ((int32_t)b[2] << 10) | ((int32_t)b[3] << 2) | ((b[6] >> 4) & 3);
  int32_t z = ((int32_t)b[4] << 10) | ((int32_t)b[5] << 2) | ((b[6] >> 2) & 3);

  // 131072 counts = 0 field, subtract to make signed
  out->mag_x = x - MAG_NULL_OFFSET;
  out->mag_y = y - MAG_NULL_OFFSET;
  out->mag_z = z - MAG_NULL_OFFSET;
  out->status = b[8]; // bit0 = meas done
  return true;
}