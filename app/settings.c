#include "settings.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "sample_ring.h"

const uint16_t settings_window_options_s[SETTINGS_WINDOW_COUNT] = {1U, 2U, 5U, 10U, 30U, 60U};
const uint16_t settings_rate_options_hz[SETTINGS_RATE_COUNT] = {10U, 50U, 100U, 200U, 500U};
const uint8_t settings_refresh_options_hz[SETTINGS_REFRESH_COUNT] = {15U, 30U, 60U};

/* INA228 ADC_CONFIG: MODE[15:12]=0xF (continuous bus+shunt+temp),
 * VBUSCT[11:9], VSHCT[8:6], VTCT[5:3], AVG[2:0].
 *   FAST   : 150 us conversions, no averaging   -> ~0.45 ms per update
 *   NORMAL : 1052 us conversions, 4x averaging  -> ~12.6 ms per update (driver default)
 *   SMOOTH : 1052 us conversions, 16x averaging -> ~50 ms per update
 *   ULTRA  : 4120 us conversions, 16x averaging -> ~200 ms per update */
const uint16_t settings_filter_adc_config[SETTINGS_FILTER_COUNT] = {
    0xF000U | (2U << 9) | (2U << 6) | (2U << 3) | 0U,
    0xF000U | (5U << 9) | (5U << 6) | (5U << 3) | 1U,
    0xF000U | (5U << 9) | (5U << 6) | (5U << 3) | 2U,
    0xF000U | (7U << 9) | (7U << 6) | (7U << 3) | 2U
};

static SemaphoreHandle_t settings_mutex;
static settings_t settings_current;
static volatile uint32_t settings_gen;

static bool settings_lock(void)
{
    if (settings_mutex == NULL ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return false;
    }
    return xSemaphoreTake(settings_mutex, pdMS_TO_TICKS(50U)) == pdTRUE;
}

static void settings_unlock(bool locked)
{
    if (locked) {
        (void)xSemaphoreGive(settings_mutex);
    }
}

static uint32_t nearest_u16(const uint16_t *options, uint32_t count, uint32_t value)
{
    uint32_t best = 0U;
    uint32_t best_diff = UINT32_MAX;

    for (uint32_t i = 0U; i < count; i++) {
        uint32_t diff = (options[i] > value) ? (options[i] - value)
                                             : (value - options[i]);
        if (diff < best_diff) {
            best_diff = diff;
            best = i;
        }
    }
    return best;
}

void settings_defaults(settings_t *out)
{
    memset(out, 0, sizeof(*out));
    out->mode = SCOPE_MODE_MW;
    out->window_s = 10U;
    out->sample_rate_hz = 50U;
    out->refresh_hz = 30U;
    out->y_auto = true;
    out->y_min = 0.0f;
    out->y_max = 1000.0f;
    out->decimation = SCOPE_DECIMATE_MEAN;
    out->filter = SCOPE_FILTER_NORMAL;
}

void settings_validate(settings_t *cfg)
{
    cfg->mode = (cfg->mode == SCOPE_MODE_MA) ? SCOPE_MODE_MA : SCOPE_MODE_MW;
    cfg->window_s = settings_window_options_s[settings_window_index(cfg->window_s)];
    cfg->sample_rate_hz = settings_rate_options_hz[settings_rate_index(cfg->sample_rate_hz)];
    cfg->refresh_hz = settings_refresh_options_hz[settings_refresh_index(cfg->refresh_hz)];
    if (cfg->decimation != SCOPE_DECIMATE_MAX) {
        cfg->decimation = SCOPE_DECIMATE_MEAN;
    }
    if ((uint32_t)cfg->filter >= SETTINGS_FILTER_COUNT) {
        cfg->filter = SCOPE_FILTER_NORMAL;
    }
    if (!(cfg->y_max > cfg->y_min)) {
        cfg->y_min = 0.0f;
        cfg->y_max = 1000.0f;
    }

    /* The ring must hold one full window plus slack for the UI to lag. */
    while ((uint32_t)cfg->window_s * cfg->sample_rate_hz > SAMPLE_RING_CAPACITY - 1024U) {
        uint32_t idx = settings_window_index(cfg->window_s);
        if (idx == 0U) {
            break;
        }
        cfg->window_s = settings_window_options_s[idx - 1U];
    }
}

void settings_init(void)
{
    settings_t loaded;

    settings_defaults(&settings_current);
    if (settings_load(&loaded)) {
        settings_validate(&loaded);
        settings_current = loaded;
    }
    settings_mutex = xSemaphoreCreateMutex();
    settings_gen = 1U;
}

void settings_get(settings_t *out)
{
    bool locked = settings_lock();
    *out = settings_current;
    settings_unlock(locked);
}

void settings_set(const settings_t *cfg)
{
    settings_t next = *cfg;
    bool locked;

    settings_validate(&next);
    locked = settings_lock();
    settings_current = next;
    settings_gen++;
    settings_unlock(locked);
}

void settings_reset_defaults(void)
{
    settings_t def;
    settings_defaults(&def);
    settings_set(&def);
}

uint32_t settings_generation(void)
{
    return settings_gen;
}

uint32_t settings_window_index(uint16_t window_s)
{
    return nearest_u16(settings_window_options_s, SETTINGS_WINDOW_COUNT, window_s);
}

uint32_t settings_rate_index(uint16_t rate_hz)
{
    return nearest_u16(settings_rate_options_hz, SETTINGS_RATE_COUNT, rate_hz);
}

uint32_t settings_refresh_index(uint8_t refresh_hz)
{
    uint32_t best = 0U;
    uint32_t best_diff = UINT32_MAX;

    for (uint32_t i = 0U; i < SETTINGS_REFRESH_COUNT; i++) {
        uint32_t opt = settings_refresh_options_hz[i];
        uint32_t diff = (opt > refresh_hz) ? (opt - refresh_hz) : (refresh_hz - opt);
        if (diff < best_diff) {
            best_diff = diff;
            best = i;
        }
    }
    return best;
}

/* TODO: persistence. The project has no NVM helper yet. The linker script
 * reserves an MRAM_USER region (.mram_user) and the Ensemble pack ships an
 * MRAM driver; wire settings_load()/settings_save() to it with a
 * magic + version + CRC header. Until then settings live in RAM only. */
bool settings_load(settings_t *out)
{
    (void)out;
    return false;
}

bool settings_save(const settings_t *cfg)
{
    (void)cfg;
    return false;
}
