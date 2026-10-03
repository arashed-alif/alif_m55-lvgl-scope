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

/* VBUS moves slowly, so it is read on every Nth sample only; that and computing
 * power instead of reading POWER leave one CURRENT transfer per sample. */
#define SENSOR_VBUS_EVERY (10U)

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
    float t = (float)sensor_task_now_ms() / 1000.0f;
    float spike = (fmodf(t, 3.0f) < 0.05f) ? 220.0f : 0.0f;
    float current = 80.0f + 40.0f * sinf(6.2831853f * 0.5f * t) +
                    8.0f * sinf(6.2831853f * 7.0f * t) +
                    4.0f * demo_noise() + spike;

    sample->vbus_v = 12.0f + 0.02f * sinf(0.3f * t) + 0.002f * demo_noise();
    sample->current_ma = current;
    sample->power_mw = sample->vbus_v * current;
    return true;
}

static bool sensor_open(uint16_t rate_hz)
{
    (void)rate_hz;
    return true;
}

static void sensor_apply_rate(uint16_t rate_hz)
{
    (void)rate_hz;
}
#else
/* INA228 ADC_CONFIG per sample rate, in settings_rate_options_hz order.
 * MODE[15:12] = 0xB converts shunt and bus continuously and skips the die
 * temperature, which nothing uses (TEMPCOMP is off). VBUSCT[11:9], VSHCT[8:6],
 * VTCT[5:3] (unused), AVG[2:0]. A result takes (VBUSCT + VSHCT) x AVG.
 *
 * Each rate averages over most of its period, so a sample is the mean of that
 * slice of time instead of a snapshot that can miss spikes between reads. It
 * stays below the period so the 1 ms tick jitter on reads rarely sees the same
 * result twice:
 *   20 Hz  (50 ms): 1052 us, 16x -> 33.7 ms (67 %)
 *   100 Hz (10 ms): 1052 us,  4x ->  8.4 ms (84 %)
 *   333 Hz  (3 ms):  280 us,  4x ->  2.2 ms (75 %)
 * At 333 Hz, 84 us x 16 would fill 90 % but leave little room for jitter at a
 * 3 ms period. */
#define INA228_ADC_MODE_SHUNT_BUS (0xB000U)
#define INA228_CT_280US           (3U)
#define INA228_CT_1052US          (5U)
#define INA228_AVG_4              (1U)
#define INA228_AVG_16             (2U)
#define INA228_ADC_CONFIG(ct, avg) \
    ((uint16_t)(INA228_ADC_MODE_SHUNT_BUS | ((ct) << 9) | ((ct) << 6) | (avg)))

static const uint16_t sensor_adc_config[SETTINGS_RATE_COUNT] = {
    INA228_ADC_CONFIG(INA228_CT_1052US, INA228_AVG_16),     /* 20 Hz */
    INA228_ADC_CONFIG(INA228_CT_1052US, INA228_AVG_4),      /* 100 Hz */
    INA228_ADC_CONFIG(INA228_CT_280US, INA228_AVG_4)        /* 333 Hz */
};

static float sensor_vbus_v;
static uint32_t sensor_vbus_age = SENSOR_VBUS_EVERY;   /* forces a first read */

static bool sensor_read(sample_t *sample)
{
    int32_t current_na;

    /* Driver units: VBUS in uV, CURRENT in nA (see drv_ina228_i3c.c). A failed
     * VBUS read fails the sample and is retried on the next one. */
    if (sensor_vbus_age >= SENSOR_VBUS_EVERY) {
        uint32_t vbus_uv;

        if (INA228_ReadVBUS(sensor_device_address, &vbus_uv) != 0) {
            return false;
        }
        sensor_vbus_v = (float)vbus_uv / 1000000.0f;
        sensor_vbus_age = 0U;
    }
    if (INA228_ReadCURRENT(sensor_device_address, &current_na) != 0) {
        return false;
    }
    sensor_vbus_age++;

    sample->vbus_v = sensor_vbus_v;
    sample->current_ma = (float)current_na / 1000000.0f;
    /* The chip's POWER is the same product, but unsigned; this keeps the sign
     * of reverse current. */
    sample->power_mw = sample->vbus_v * sample->current_ma;
    return true;
}

static bool sensor_open(uint16_t rate_hz)
{
    uint16_t ina_config = 0x0010U;   /* ADCRANGE = 1 (±40.96 mV shunt) */

    if (INA228_Init(&sensor_device_address, 1U) != 0 ||
        INA228_WriteCONFIG(sensor_device_address, &ina_config) != 0 ||
        INA228_Write(sensor_device_address, ADC_CONFIG,
                     sensor_adc_config[settings_rate_index(rate_hz)]) != 0 ||
        INA228_WriteSHUNT_CAL(sensor_device_address, 1022U) != 0) {
        return false;
    }
    return true;
}

static void sensor_apply_rate(uint16_t rate_hz)
{
    if (INA228_Write(sensor_device_address, ADC_CONFIG,
                     sensor_adc_config[settings_rate_index(rate_hz)]) != 0) {
        sensor_errors++;
    }
}
#endif

/* Whole milliseconds, truncated: 333 Hz runs at 3 ms, i.e. 333.3 Hz. */
static uint32_t sensor_period_for(uint16_t rate_hz)
{
    uint32_t period_ms = 1000U / rate_hz;

    return (period_ms == 0U) ? 1U : period_ms;
}

static void sensor_task(void *argument)
{
    TickType_t next_wake;
    uint32_t seen_generation = settings_generation();
    uint32_t rate_window_start = sensor_task_now_ms();
    uint32_t rate_window_count = 0U;
    uint32_t period_ms;
    uint16_t active_rate;
    settings_t cfg;

    (void)argument;

    /* Open with the configured rate so the first conversions already use its
     * averaging. */
    settings_get(&cfg);
    active_rate = cfg.sample_rate_hz;
    period_ms = sensor_period_for(active_rate);
    sensor_period_ms = period_ms;

    if (!sensor_open(active_rate)) {
        sensor_errors++;
        sensor_task_handle = NULL;
        vTaskDelete(NULL);
    }

    /* Let the first averaged result complete: it always fits in one period. */
    vTaskDelay(pdMS_TO_TICKS((period_ms > 15U) ? period_ms : 15U));
    next_wake = xTaskGetTickCount();

    for (;;) {
        sample_t sample;
        uint32_t generation = settings_generation();

        if (generation != seen_generation) {
            settings_get(&cfg);
            seen_generation = generation;
            if (cfg.sample_rate_hz != active_rate) {
                active_rate = cfg.sample_rate_hz;
                period_ms = sensor_period_for(active_rate);
                sensor_period_ms = period_ms;
                sensor_apply_rate(active_rate);
            }
        }

        if (sensor_read(&sample)) {
            sample.t_ms = sensor_task_now_ms();
            (void)sample_ring_push(&sample_ring, &sample);
            sensor_sample_count++;
            rate_window_count++;
        } else {
            sensor_errors++;
        }

        if ((sensor_task_now_ms() - rate_window_start) >= 1000U) {
            sensor_achieved_hz = rate_window_count;
            rate_window_count = 0U;
            rate_window_start = sensor_task_now_ms();
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

uint32_t sensor_task_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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
