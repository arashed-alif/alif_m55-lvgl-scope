#include "sample_ring.h"

static uint32_t ring_load(const volatile uint32_t *value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

static void ring_store(volatile uint32_t *value, uint32_t next)
{
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

void sample_ring_init(sample_ring_t *ring)
{
    ring_store(&ring->head, 0U);
    ring_store(&ring->tail, 0U);
    ring_store(&ring->dropped, 0U);
}

bool sample_ring_push(sample_ring_t *ring, const sample_t *sample)
{
    uint32_t head = ring_load(&ring->head);
    uint32_t tail = ring_load(&ring->tail);

    if ((head - tail) >= SAMPLE_RING_CAPACITY) {
        __atomic_fetch_add(&ring->dropped, 1U, __ATOMIC_RELAXED);
        return false;
    }

    ring->samples[head & SAMPLE_RING_INDEX_MASK] = *sample;
    ring_store(&ring->head, head + 1U);
    return true;
}

uint32_t sample_ring_head(const sample_ring_t *ring)
{
    return ring_load(&ring->head);
}

uint32_t sample_ring_tail(const sample_ring_t *ring)
{
    return ring_load(&ring->tail);
}

bool sample_ring_get(const sample_ring_t *ring, uint32_t seq, sample_t *sample)
{
    uint32_t head = ring_load(&ring->head);
    uint32_t tail = ring_load(&ring->tail);

    /* Unsigned wrap-safe check for tail <= seq < head. */
    if ((seq - tail) >= (head - tail)) {
        return false;
    }

    *sample = ring->samples[seq & SAMPLE_RING_INDEX_MASK];
    return true;
}

void sample_ring_consume(sample_ring_t *ring, uint32_t new_tail)
{
    uint32_t head = ring_load(&ring->head);
    uint32_t tail = ring_load(&ring->tail);

    /* Only move forward, and never past head. */
    if ((new_tail - tail) > (head - tail)) {
        new_tail = head;
    }
    if (new_tail != tail) {
        ring_store(&ring->tail, new_tail);
    }
}

bool sample_ring_pop(sample_ring_t *ring, sample_t *sample)
{
    uint32_t tail = ring_load(&ring->tail);
    uint32_t head = ring_load(&ring->head);

    if (tail == head) {
        return false;
    }

    *sample = ring->samples[tail & SAMPLE_RING_INDEX_MASK];
    ring_store(&ring->tail, tail + 1U);
    return true;
}

uint32_t sample_ring_count(const sample_ring_t *ring)
{
    return ring_load(&ring->head) - ring_load(&ring->tail);
}

uint32_t sample_ring_dropped(const sample_ring_t *ring)
{
    return ring_load(&ring->dropped);
}

void sample_ring_clear_dropped(sample_ring_t *ring)
{
    ring_store(&ring->dropped, 0U);
}
