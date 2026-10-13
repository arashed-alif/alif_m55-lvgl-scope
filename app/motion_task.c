#include "motion_task.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "backlight.h"
#include "drv_bmi323.h"
#include "settings.h"

/* 1: print motion and backlight transitions (they are rare). */
#define MOTION_VERBOSE          1

/* Detector tuning (feature engine units: 1/512 g, 20 ms per sample).
 * Picked up: 60 mg of change for 60 ms. Set down: under 20 mg for 1 s.
 * Raise MOTION_ANY_THRESHOLD if desk bumps wake the screen; raise
 * MOTION_NO_THRESHOLD if it dims while held in a hand. */
#define MOTION_ANY_THRESHOLD    31U
#define MOTION_ANY_HYSTERESIS   5U
#define MOTION_ANY_DURATION     3U
#define MOTION_NO_THRESHOLD     10U
#define MOTION_NO_HYSTERESIS    2U
#define MOTION_NO_DURATION      50U
#define MOTION_NO_DURATION_MS   (MOTION_NO_DURATION * 20U)

/* Fallback poll of the latched interrupt status, in case an INT1 edge is
 * missed (or the IRQ GPIO in board_defs.h is not the one INT1 is wired to). */
#define MOTION_POLL_MS          500U

static SemaphoreHandle_t wake_sem;
static TaskHandle_t motion_task_handle;

static volatile uint32_t last_touch_ms;
static volatile bool motion_dimmed;
static volatile bool motion_still;
static volatile bool motion_imu_ok;
static volatile uint32_t motion_imu_errors;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* INT1 falling edge (ISR context). */
static void motion_irq(void)
{
    BaseType_t woken = pdFALSE;

    (void)xSemaphoreGiveFromISR(wake_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

static bool imu_open(void)
{
    const bmi323_motion_config_t motion_cfg = {
        MOTION_ANY_THRESHOLD, MOTION_ANY_HYSTERESIS, MOTION_ANY_DURATION,
        MOTION_NO_THRESHOLD, MOTION_NO_HYSTERESIS, MOTION_NO_DURATION
    };
    uint16_t status;

    if (BMI323_Init(BMI323_I2C_ADDR) != 0 ||
        BMI323_ConfigMotion(BMI323_I2C_ADDR, &motion_cfg) != 0) {
        return false;
    }
    /* Start out listening for "set down"; clear anything latched first. */
    if (BMI323_MapInt1(BMI323_I2C_ADDR, BMI323_INT_NO_MOTION) != 0 ||
        BMI323_ReadIntStatus1(BMI323_I2C_ADDR, &status) != 0) {
        return false;
    }
    /* Without the IRQ the poll still works, only slower. */
    (void)BMI323_IrqInit(motion_irq);
    return true;
}

/* Read the latched detector status and update motion_still. Only one detector
 * is mapped at a time: no-motion while moving, any-motion while still, so the
 * line fires once per transition instead of on every 20 ms evaluation. */
static void imu_poll(uint32_t *still_since)
{
    uint16_t status = 0U;

    if (BMI323_ReadIntStatus1(BMI323_I2C_ADDR, &status) != 0) {
        motion_imu_errors++;
        return;
    }

    if (!motion_still && (status & BMI323_INT_NO_MOTION) != 0U) {
        motion_still = true;
        *still_since = now_ms() - MOTION_NO_DURATION_MS;
        if (BMI323_MapInt1(BMI323_I2C_ADDR, BMI323_INT_ANY_MOTION) != 0) {
            motion_imu_errors++;
        }
#if MOTION_VERBOSE
        printf("motion: set down\n");
#endif
    } else if (motion_still && (status & BMI323_INT_ANY_MOTION) != 0U) {
        motion_still = false;
        if (BMI323_MapInt1(BMI323_I2C_ADDR, BMI323_INT_NO_MOTION) != 0) {
            motion_imu_errors++;
        }
#if MOTION_VERBOSE
        printf("motion: picked up\n");
#endif
    }
}

static void motion_task(void *argument)
{
    uint32_t seen_generation = 0U;
    uint32_t still_since = now_ms();
    settings_t cfg;

    (void)argument;

    settings_get(&cfg);
    seen_generation = settings_generation();

    motion_imu_ok = imu_open();
    if (!motion_imu_ok) {
        /* Touch-only: treat the board as always still, so it dims after
         * dim_after_s without a touch and wakes on touch. */
        printf("motion: BMI323 unavailable, auto-dim on touch only\n");
        motion_still = true;
    }

    for (;;) {
        uint32_t generation = settings_generation();
        uint32_t now;
        uint32_t idle_ms;
        uint32_t dim_after_ms;
        uint32_t wait_ms = MOTION_POLL_MS;
        bool want_dim;
        uint8_t level;

        if (generation != seen_generation) {
            settings_get(&cfg);
            seen_generation = generation;
        }

        if (motion_imu_ok) {
            imu_poll(&still_since);
        }

        /* Idle = time since the later of "became still" and "last touch". */
        now = now_ms();
        idle_ms = now - still_since;
        if (now - last_touch_ms < idle_ms) {
            idle_ms = now - last_touch_ms;
        }
        dim_after_ms = (uint32_t)cfg.dim_after_s * 1000U;
        want_dim = cfg.auto_dim && motion_still && idle_ms >= dim_after_ms;

        level = (uint8_t)(want_dim ? cfg.dim_pct : cfg.bright_pct);
        if (level != backlight_target()) {
            backlight_fade_to(level, BACKLIGHT_FADE_MS);
#if MOTION_VERBOSE
            printf("motion: backlight %u%%\n", (unsigned int)level);
#endif
        }
        motion_dimmed = want_dim;

        /* Wake for the dim deadline if it comes before the next poll. */
        if (cfg.auto_dim && motion_still && !want_dim &&
            dim_after_ms - idle_ms < wait_ms) {
            wait_ms = dim_after_ms - idle_ms;
        }
        (void)xSemaphoreTake(wake_sem, pdMS_TO_TICKS(wait_ms) + 1U);
    }
}

bool motion_task_start(uint32_t priority)
{
    if (motion_task_handle != NULL) {
        return false;
    }

    wake_sem = xSemaphoreCreateBinary();
    if (wake_sem == NULL) {
        return false;
    }
    last_touch_ms = now_ms();

    if (xTaskCreate(motion_task, "Motion", MOTION_TASK_STACK_WORDS, NULL,
                    (UBaseType_t)priority, &motion_task_handle) != pdPASS) {
        motion_task_handle = NULL;
        return false;
    }
    return true;
}

bool motion_task_touch(void)
{
    last_touch_ms = now_ms();
    if (motion_dimmed) {
        (void)xSemaphoreGive(wake_sem);
        return true;
    }
    return false;
}

void motion_task_get_status(motion_status_t *out)
{
    out->imu_ok = motion_imu_ok;
    out->still = motion_still;
    out->dimmed = motion_dimmed;
    out->imu_errors = motion_imu_errors;
}
