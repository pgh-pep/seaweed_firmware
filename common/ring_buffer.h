#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t *buf;
    uint32_t mask;
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t overflow;
} ring_buf_t;

bool init_ring(ring_buf_t *r, uint8_t *buffer, uint32_t size);
void push_ring(ring_buf_t *r, const uint8_t *data, uint32_t len);
bool pop_ring(ring_buf_t *r, uint8_t *out);
bool is_ring_empty(const ring_buf_t *r);