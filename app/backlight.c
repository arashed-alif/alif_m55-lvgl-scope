#include "backlight.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "utimer_pwm.h"

#define BACKLIGHT_STEP_MS (10U)

/* Fade state, shared between backlight_fade_to() (caller's task) and the
 * timer callback (timer service task); both touch it inside critical
 * sections. Levels are in PWM permille. */
static TimerHandle_t fade_timer;
static uint16_t level_now;
static uint16_t level_from;
static uint16_t level_to;
static uint32_t fade_steps;
static uint32_t fade_step;
static volatile uint8_t target_percent;

static uint16_t percent_to_permille(uint8_t percent)
{
    if (percent > 100U) {
        percent = 100U;
    }
    return (uint16_t)(percent * (UTIMER_PWM_DUTY_MAX / 100U));
}

static void fade_timer_cb(TimerHandle_t timer)
{
    uint16_t level;
    bool done;

    taskENTER_CRITICAL();
    fade_step++;
    done = (fade_step >= fade_steps);
    if (done) {
        level = level_to;
    } else {
        int32_t span = (int32_t)level_to - (int32_t)level_from;
        level = (uint16_t)((int32_t)level_from + (span * (int32_t)fade_step) / (int32_t)fade_steps);
    }
    level_now = level;
    taskEXIT_CRITICAL();

    utimer_pwm_set_duty(level);
    if (done) {
        (void)xTimerStop(timer, 0);
    }
}

void backlight_init(uint8_t percent)
{
    level_now = percent_to_permille(percent);
    level_to = level_now;
    target_percent = percent;
    utimer_pwm_init(level_now);

    fade_timer = xTimerCreate("backlight", pdMS_TO_TICKS(BACKLIGHT_STEP_MS),
                              pdTRUE, NULL, fade_timer_cb);
}

void backlight_fade_to(uint8_t percent, uint32_t fade_ms)
{
    uint16_t level = percent_to_permille(percent);
    uint32_t steps = fade_ms / BACKLIGHT_STEP_MS;

    target_percent = percent;

    if (fade_timer == NULL || steps == 0U) {
        if (fade_timer != NULL) {
            (void)xTimerStop(fade_timer, 0);
        }
        /* fade_steps = 0 makes a timer tick already queued land on level. */
        taskENTER_CRITICAL();
        level_now = level;
        level_to = level;
        fade_steps = 0U;
        taskEXIT_CRITICAL();
        utimer_pwm_set_duty(level);
        return;
    }

    taskENTER_CRITICAL();
    level_from = level_now;
    level_to = level;
    fade_steps = steps;
    fade_step = 0U;
    taskEXIT_CRITICAL();

    /* Starting a running timer just restarts its period. */
    (void)xTimerStart(fade_timer, 0);
}

uint8_t backlight_target(void)
{
    return target_percent;
}
