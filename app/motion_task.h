#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 512 words = 2 KB. Verify with uxTaskGetStackHighWaterMark() under GDB. */
#define MOTION_TASK_STACK_WORDS (512U)

typedef struct {
    bool imu_ok;            /* BMI323 found and configured */
    bool still;             /* no-motion reported, no any-motion since */
    bool dimmed;            /* backlight at (or fading to) the dim level */
    uint32_t imu_errors;    /* failed BMI323 transactions */
} motion_status_t;

/* Auto-dim: watches the BMI323 motion detectors and touch activity and fades
 * the backlight between the bright and dim levels in settings. backlight_init()
 * and i3c_bus_init() must have run. */
bool motion_task_start(uint32_t priority);

/* Call from the touch read path on every pressed sample. Returns true while
 * the screen is dimmed: that touch should only wake the screen, not reach the
 * UI. Never blocks. */
bool motion_task_touch(void);

void motion_task_get_status(motion_status_t *out);

#ifdef __cplusplus
}
#endif

#endif
