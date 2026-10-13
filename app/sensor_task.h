#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "sample_ring.h"

#define SENSOR_TASK_DEFAULT_PERIOD_MS (10U)
/* 512 words = 2 KB. Verify with uxTaskGetStackHighWaterMark() under GDB. */
#define SENSOR_TASK_STACK_WORDS (512U)

typedef struct {
    uint32_t period_ms;     /* initial period; overridden by settings once running */
    uint32_t priority;
} sensor_task_config_t;

typedef struct {
    uint32_t samples;       /* total samples pushed */
    uint32_t errors;        /* failed INA228 transactions */
    uint32_t dropped;       /* ring overruns */
    uint32_t achieved_hz;   /* samples pushed during the last full second */
    uint32_t period_ms;     /* period currently requested from settings */
    bool demo;              /* true when built with UI_DEMO_DATA */
} sensor_stats_t;

/* Samples every INA228 in sensor_channels[] (sensor_task.c) into one ring. */
bool sensor_task_start(const sensor_task_config_t *config);
sample_ring_t *sensor_task_samples(void);
/* Current time on the clock that stamps sample_t.t_ms. */
uint32_t sensor_task_now_ms(void);
void sensor_task_get_stats(sensor_stats_t *out);
uint32_t sensor_task_error_count(void);

/* Per channel (sample_t.ch index): the INA228's I2C address, and whether it
 * answered at start-up. A channel that isn't present, or is switched off in
 * settings, carries NAN in every sample. */
uint8_t sensor_task_channel_addr(uint32_t ch);
bool sensor_task_channel_present(uint32_t ch);

#endif
