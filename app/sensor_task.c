#include "sensor_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "settings.h"

#ifndef UI_DEMO_DATA
#define UI_DEMO_DATA 0
#endif

#if UI_DEMO_DATA
#include <math.h>
#else
#include "drv_ina228.h"
#endif

/* 512 KB; lands in SRAM1 (see linker .bss.at_sram1), away from the LCD
 * frame buffers which live in SRAM0. */
static sample_ring_t sample_ring
    __attribute__((section(".bss.at_sram1"), aligned(32)));

static uint8_t sensor_device_address;
static volatile uint32_t sensor_errors;
static volatile uint32_t sensor_sample_count;
static volatile uint32_t sensor_achieved_hz;
static volatile uint32_t sensor_period_ms = SENSOR_TASK_DEFAULT_PERIOD_MS;
static sensor_task_config_t sensor_config = {
    SENSOR_TASK_DEFAULT_PERIOD_MS,
    tskIDLE_PRIORITY + 3U
};
static TaskHandle_t sensor_task_handle;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

#if UI_DEMO_DATA
/* Synthetic source: slow sine + ripple + noise + a spike every ~3 s. */
static uint32_t demo_rng = 0x2545F491U;

static float demo_noise(void)
{
    demo_rng = demo_rng * 1664525U + 1013904223U;
    return ((float)(demo_rng >> 8) / 16777216.0f) - 0.5f;
}

static bool sensor_read(sample_t *sample)
{
    float t = (float)now_ms() / 1000.0f;
    float spike = (fmodf(t, 3.0f) < 0.05f) ? 220.0f : 0.0f;
    float current = 80.0f + 40.0f * sinf(6.2831853f * 0.5f * t) +
                    8.0f * sinf(6.2831853f * 7.0f * t) +
                    4.0f * demo_noise() + spike;

    sample->vbus_v = 12.0f + 0.02f * sinf(0.3f * t) + 0.002f * demo_noise();
    sample->current_ma = current;
    sample->power_mw = sample->vbus_v * current;
    return true;
}

static bool sensor_open(void)
{
    return true;
}

static void sensor_apply_filter(scope_filter_t filter)
{
    (void)filter;
}
#else
static bool sensor_read(sample_t *sample)
{
    uint32_t vbus_uv;
    int32_t current_na;
    uint64_t power_nw;
    int32_t result;

    /* Driver units: VBUS in uV, CURRENT in nA, POWER in nW (see drv_ina228_i3c.c). */
    result = INA228_ReadVBUS(sensor_device_address, &vbus_uv);
    result |= INA228_ReadCURRENT(sensor_device_address, &current_na);
    result |= INA228_ReadPOWER(sensor_device_address, &power_nw);
    if (result != 0) {
        return false;
    }

    sample->vbus_v = (float)vbus_uv / 1000000.0f;
    sample->current_ma = (float)current_na / 1000000.0f;
    sample->power_mw = (float)power_nw / 1000000.0f;
    return true;
}

static bool sensor_open(void)
{
    uint16_t ina_config = 0x0010U;   /* ADCRANGE = 1 (±40.96 mV shunt) */

    if (INA228_Init(&sensor_device_address, 1U) != 0 ||
        INA228_WriteCONFIG(sensor_device_address, &ina_config) != 0 ||
        INA228_Write(sensor_device_address, ADC_CONFIG,
                     settings_filter_adc_config[SCOPE_FILTER_NORMAL]) != 0 ||
        INA228_WriteSHUNT_CAL(sensor_device_address, 48U) != 0) {
        return false;
    }
    return true;
}

static void sensor_apply_filter(scope_filter_t filter)
{
    if (INA228_Write(sensor_device_address, ADC_CONFIG,
                     settings_filter_adc_config[filter]) != 0) {
        sensor_errors++;
    }
}
#endif

static void sensor_task(void *argument)
{
    TickType_t next_wake;
    uint32_t seen_generation = 0U;
    uint32_t rate_window_start = now_ms();
    uint32_t rate_window_count = 0U;
    scope_filter_t active_filter = SCOPE_FILTER_NORMAL;
    uint32_t period_ms = sensor_config.period_ms;

    (void)argument;

    if (!sensor_open()) {
        sensor_errors++;
        sensor_task_handle = NULL;
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(15U));
    next_wake = xTaskGetTickCount();

    for (;;) {
        sample_t sample;
        uint32_t generation = settings_generation();

        if (generation != seen_generation) {
            settings_t cfg;

            settings_get(&cfg);
            seen_generation = generation;
            period_ms = 1000U / cfg.sample_rate_hz;
            if (period_ms == 0U) {
                period_ms = 1U;
            }
            sensor_period_ms = period_ms;
            if (cfg.filter != active_filter) {
                active_filter = cfg.filter;
                sensor_apply_filter(active_filter);
            }
        }

        if (sensor_read(&sample)) {
            sample.t_ms = now_ms();
            (void)sample_ring_push(&sample_ring, &sample);
            sensor_sample_count++;
            rate_window_count++;
        } else {
            sensor_errors++;
        }

        if ((now_ms() - rate_window_start) >= 1000U) {
            sensor_achieved_hz = rate_window_count;
            rate_window_count = 0U;
            rate_window_start = now_ms();
        }

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(period_ms));
    }
}

bool sensor_task_start(uint8_t dev_addr, const sensor_task_config_t *config)
{
    if (sensor_task_handle != NULL || config == NULL ||
        config->period_ms == 0U) {
        return false;
    }

    sensor_device_address = dev_addr;
    sensor_config = *config;
    sensor_errors = 0U;
    sensor_sample_count = 0U;
    sensor_achieved_hz = 0U;
    sample_ring_init(&sample_ring);

    if (xTaskCreate(sensor_task, "Sensor", SENSOR_TASK_STACK_WORDS, NULL,
                    (UBaseType_t)sensor_config.priority,
                    &sensor_task_handle) != pdPASS) {
        sensor_task_handle = NULL;
        return false;
    }

    return true;
}

sample_ring_t *sensor_task_samples(void)
{
    return &sample_ring;
}

void sensor_task_get_stats(sensor_stats_t *out)
{
    out->samples = sensor_sample_count;
    out->errors = sensor_errors;
    out->dropped = sample_ring_dropped(&sample_ring);
    out->achieved_hz = sensor_achieved_hz;
    out->period_ms = sensor_period_ms;
    out->demo = (UI_DEMO_DATA != 0);
}

uint32_t sensor_task_error_count(void)
{
    return sensor_errors;
}
