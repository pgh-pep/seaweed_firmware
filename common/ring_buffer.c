#include "ring_buffer.h"

bool init_ring(ring_buf_t *r, uint8_t *buffer, uint32_t size) {
  // make sure size is pow of 2 (since wrapping w/ mask)
  if (size < 2 || (size & (size - 1)) != 0)
    return false;

  r->buf = buffer;
  r->mask = size - 1u;

  r->overflow = 0;
  r->tail = 0;
  r->head = 0;

  return true;
}

bool is_ring_empty(const ring_buf_t *r) { return r->head == r->tail; }

void push_ring(ring_buf_t *r, const uint8_t *data, uint32_t len) {
  uint32_t h = r->head;
  uint32_t t = r->tail;
  uint32_t free = (t - h - 1u) & r->mask;

  if (len > free) {
    r->overflow += len;
    return;
  }
  for (uint32_t i = 0; i < len; i++) {
    r->buf[h] = data[i];
    h = (h + 1u) & r->mask;
  }
  r->head = h;
}

bool pop_ring(ring_buf_t *r, uint8_t *out) {
  uint32_t t = r->tail;
  if (t == r->head)
    return false;
  *out = r->buf[t];
  r->tail = (t + 1u) & r->mask;
  return true;
}