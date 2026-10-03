#ifndef SAMPLE_RING_H
#define SAMPLE_RING_H

#include <stdbool.h>
#include <stdint.h>

/* 32768 x 16 B = 512 KB. Sized for 60 s at 333 Hz (20000) plus slack. */
#define SAMPLE_RING_CAPACITY (32768U)
#define SAMPLE_RING_INDEX_MASK (SAMPLE_RING_CAPACITY - 1U)
/* Space the consumer always leaves free so the producer can keep pushing while
 * the UI is between ticks. */
#define SAMPLE_RING_HEADROOM (1024U)

typedef struct {
    uint32_t t_ms;
    float vbus_v;
    float current_ma;
    float power_mw;
} sample_t;

/* Single producer (sensor task) / single consumer (UI timer).
 * head = sequence number of the next sample to be written,
 * tail = oldest sequence number the consumer still wants retained.
 * Samples in [tail, head) are valid and never overwritten by the producer. */
typedef struct {
    sample_t samples[SAMPLE_RING_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t dropped;
} sample_ring_t;

void sample_ring_init(sample_ring_t *ring);

/* Producer side. Returns false (and counts a drop) if the ring is full. */
bool sample_ring_push(sample_ring_t *ring, const sample_t *sample);

/* Consumer side. */
uint32_t sample_ring_head(const sample_ring_t *ring);
uint32_t sample_ring_tail(const sample_ring_t *ring);
/* Copy the sample with sequence number `seq`; false if it is outside [tail, head). */
bool sample_ring_get(const sample_ring_t *ring, uint32_t seq, sample_t *sample);
/* First sequence number in [tail, head) whose t_ms is at or after t_ms, or head
 * if there is none. Pass the head the caller is working with. Timestamps never
 * decrease, so this is a binary search. */
uint32_t sample_ring_seek(const sample_ring_t *ring, uint32_t head, uint32_t t_ms);
/* Advance tail so that only [new_tail, head) stays retained. Never moves backwards. */
void sample_ring_consume(sample_ring_t *ring, uint32_t new_tail);
/* Classic pop of the oldest retained sample (kept for convenience/tests). */
bool sample_ring_pop(sample_ring_t *ring, sample_t *sample);

uint32_t sample_ring_count(const sample_ring_t *ring);
uint32_t sample_ring_dropped(const sample_ring_t *ring);
void sample_ring_clear_dropped(sample_ring_t *ring);

#endif
