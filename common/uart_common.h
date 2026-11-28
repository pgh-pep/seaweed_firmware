#ifndef UART_COMMON_H
#define UART_COMMON_H

#include "common_types.h"
#include <stddef.h>
#include <stdint.h>

// frame (header + payload + CRC) needs to be smaller
// than 254B for COBS (w/ single code byte)
#define PKT_MAX_DECODED 254u
#define PKT_HEADER_LEN 2u
#define PKT_CRC_LEN 2u
#define PKT_MAX_PAYLOAD (PKT_MAX_DECODED - PKT_HEADER_LEN - PKT_CRC_LEN) // 250B

#define PKT_MAX_ENCODED (PKT_MAX_DECODED + 1u) // 255B, incl COBS code byte
#define PKT_MAX_RAW (PKT_MAX_ENCODED + 1u)     // 256B, incl 0x00 delimiter

#define PKT_DELIMITER 0x00u

// pkt struct: [header] [payload] [crc]

// PACKETS
// stm to jetson (tx)
#pragma pack(push, 1)
typedef struct {
  uint32_t stamp_ms;
  imu_data_t imu_data;
  mag_data_t mag_data;
  uint16_t voltage;
  int32_t current;
} packet_tx_t;
#pragma pack(pop)

// jetson to stm (rx)
#pragma pack(push, 1)
typedef struct {
  boat_state_t state;
  thruster_commands_t thruster_commands;
  launch_actions_t launch_action;
  uint16_t launcher_angle; // degrees
} packet_rx_t;
#pragma pack(pop)

_Static_assert(sizeof(packet_tx_t) <= PKT_MAX_PAYLOAD, "packet_tx_t too big");
_Static_assert(sizeof(packet_rx_t) <= PKT_MAX_PAYLOAD, "packet_rx_t too big");

// packet header
#pragma pack(push, 1)
typedef struct {
  uint8_t counter; // counter, wrap at 255
  uint8_t size;    // payload size in bytes
} header_t;
#pragma pack(pop)

// entire frame structs:
#pragma pack(push, 1)
typedef struct {
  header_t header;
  packet_tx_t payload;
  uint16_t crc;
} frame_tx_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  header_t header;
  packet_rx_t payload;
  uint16_t crc;
} frame_rx_t;
#pragma pack(pop)

_Static_assert(sizeof(frame_tx_t) <= PKT_MAX_DECODED, "frame_tx_t too big");
_Static_assert(sizeof(frame_rx_t) <= PKT_MAX_DECODED, "frame_rx_t too big");

// CRC, poly = 0x1021, init = 0xFFFF
uint16_t crc16(const uint8_t *data, size_t len);

// COBS, return encoded length
size_t cobs_encode(const uint8_t *buf_in, size_t len, uint8_t *buf_out);

// returns decoded length (0 if incorrect)
size_t cobs_decode(const uint8_t *buf_in, size_t len, uint8_t *buf_out);

// build package w/ header, payload, CRC
// encodes and applies delimiter
// returns size of packet (0 if issue)
size_t serialize_packet(const void *payload, uint8_t payload_len,
                        uint8_t counter, uint8_t *output_buf);

typedef enum {
  PKT_OK = 0,
  PKT_ERR_COBS, // COBS does not make sense
  PKT_ERR_CRC,  // CRC checksum incorrect
  PKT_ERR_LEN,  // decoded length != header + sizeof(payload) + crc
} packet_decoding_status_t;

// validate and copy payload from frame
packet_decoding_status_t deserialize_packet(const uint8_t *in, size_t in_len,
                                            void *payload_out,
                                            uint8_t expected_len,
                                            uint8_t *counter_buf);

#define SERIALIZE_PKT(payload, counter, out)                                   \
  serialize_packet((payload), (uint8_t)sizeof(*(payload)), (counter), (out))

#define DESERIALIZE_PKT(in, in_len, p, counter_buf)                            \
  deserialize_packet((in), (in_len), (p), (uint8_t)sizeof(*(p)), (counter_buf))

#endif // UART_COMMON_H