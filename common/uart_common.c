#include <string.h>

#include "crc.h"
#include "uart_common.h"

uint16_t crc16(const uint8_t *data, size_t len) {
  return (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t *)data, len);
}

size_t serialize_packet(const void *payload, uint8_t payload_len,
                        uint8_t counter, uint8_t *output_buf) {
  // [header] [payload] [crc]
  uint8_t raw[PKT_MAX_DECODED];
  size_t crc_offset = PKT_HEADER_LEN + (size_t)payload_len;

  if (payload_len > PKT_MAX_PAYLOAD)
    return 0; // val payload length

  // serialize
  raw[0] = counter;
  raw[1] = payload_len;
  memcpy(&raw[PKT_HEADER_LEN], payload, payload_len);

  // create crc
  uint16_t crc = crc16(raw, crc_offset);
  raw[crc_offset] = (uint8_t)(crc & 0xFFu); // little-endian
  raw[crc_offset + 1] = (uint8_t)(crc >> 8);

  // encode
  size_t n = cobs_encode(raw, crc_offset + PKT_CRC_LEN, output_buf);
  output_buf[n++] = PKT_DELIMITER;
  return n;
}

packet_decoding_status_t deserialize_packet(const uint8_t *in, size_t in_len,
                                            void *payload_out,
                                            uint8_t expected_len,
                                            uint8_t *counter_buf) {
  uint8_t buf[PKT_MAX_DECODED];
  size_t crc_offset = PKT_HEADER_LEN + (size_t)expected_len;

  if (in_len == 0 || in_len > PKT_MAX_ENCODED)
    return PKT_ERR_COBS;

  // decode
  size_t n = cobs_decode(in, in_len, buf);
  // val length
  if (buf[1] != expected_len)
    return PKT_ERR_LEN;
  // 0 -> unable to decode
  if (n == 0)
    return PKT_ERR_COBS;
  // n must = total packet size
  if (n != crc_offset + PKT_CRC_LEN)
    return PKT_ERR_LEN;

  // crc check
  uint16_t rx_crc =
      (uint16_t)(buf[crc_offset] | ((uint16_t)buf[crc_offset + 1] << 8));
  if (crc16(buf, crc_offset) != rx_crc)
    return PKT_ERR_CRC;

  // save counter
  if (counter_buf) {
    *counter_buf = buf[0];
  }

  memcpy(payload_out, &buf[PKT_HEADER_LEN], expected_len);
  return PKT_OK;
}

size_t cobs_encode(const uint8_t *buf_in, size_t len, uint8_t *buf_out) {
  size_t read_idx = 0;
  size_t write_idx = 1; // idx 0 for first code byte, start at 1
  size_t code_idx = 0;  // idx of curr block's code byte
  uint8_t code = 1;     // 1 + num of non-zero bytes per block

  while (read_idx < len) {
    if (buf_in[read_idx] == 0) {
      // end of block, start new code byte
      buf_out[code_idx] = code;
      code_idx = write_idx++;
      code = 1;
      read_idx++;
    } else {
      buf_out[write_idx++] = buf_in[read_idx++];
      code++;
      // block full (254 bytes), start new block if more data left
      if (code == 0xFF && read_idx < len) {
        buf_out[code_idx] = code;
        code_idx = write_idx++;
        code = 1;
      }
    }
  }
  // close last block
  buf_out[code_idx] = code;
  return write_idx;
}

// returns decoded len, 0 if issues (empty frame returns 0)
size_t cobs_decode(const uint8_t *buf_in, size_t len, uint8_t *buf_out) {
  size_t read_idx = 0;
  size_t write_idx = 0;

  while (read_idx < len) {
    uint8_t code = buf_in[read_idx++];
    if (code == 0)
      return 0; // not poss for 0 size

    size_t n = (size_t)code - 1; // no data bytes in block
    if (read_idx + n > len)
      return 0; // block cant extend past end of frame

    for (size_t i = 0; i < n; i++) {
      uint8_t b = buf_in[read_idx++];
      if (b == 0)
        return 0; // cant have zero inside block
      buf_out[write_idx++] = b;
    }

    // put zeros back
    if (code != 0xFF && read_idx < len) {
      buf_out[write_idx++] = 0;
    }
  }

  return write_idx;
}
