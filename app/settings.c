#include "settings.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "drv_bkram.h"
#include "sample_ring.h"

/* Persistence: one record in the 4 KB backup SRAM, word-addressed through
 * drv_bkram. Indices 0-7 are claimed in drv_bkram.h, so start well clear.
 * Fields are stored one per word rather than as a raw settings_t so padding
 * and enum sizes don't matter; the CRC is written last, so a reset mid-write,
 * or the random contents after a cold start, just reads back as invalid. */
#define SETTINGS_BKRAM_BASE (32U)
#define SETTINGS_MAGIC      (0x50534331U)   /* "PSC1" */
#define SETTINGS_VERSION    (1U)            /* bump when the fields change */

enum {
    SETTINGS_W_MAGIC,
    SETTINGS_W_VERSION,
    SETTINGS_W_MODE,
    SETTINGS_W_WINDOW,
    SETTINGS_W_RATE,
    SETTINGS_W_Y_AUTO,
    SETTINGS_W_Y_MIN,
    SETTINGS_W_Y_MAX,
    SETTINGS_W_DECIMATION,
    SETTINGS_W_CRC,
    SETTINGS_W_COUNT
};

const uint16_t settings_window_options_s[SETTINGS_WINDOW_COUNT] = {1U, 2U, 5U, 10U, 30U, 60U};
/* Ascending; sensor_task.c keeps a matching ADC_CONFIG table. The sensor paces
 * in whole 1 ms ticks, so 333 Hz is a 3 ms period. A 500 Hz rate saved by an
 * older build snaps to 333 Hz in settings_validate(). */
const uint16_t settings_rate_options_hz[SETTINGS_RATE_COUNT] = {20U, 100U, 333U};

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
    out->sample_rate_hz = 100U;
    out->y_auto = true;
    out->y_min = 0.0f;
    out->y_max = 1000.0f;
    out->decimation = SCOPE_DECIMATE_MEAN;
}

void settings_validate(settings_t *cfg)
{
    cfg->mode = (cfg->mode == SCOPE_MODE_MA) ? SCOPE_MODE_MA : SCOPE_MODE_MW;
    cfg->window_s = settings_window_options_s[settings_window_index(cfg->window_s)];
    cfg->sample_rate_hz = settings_rate_options_hz[settings_rate_index(cfg->sample_rate_hz)];
    if (cfg->decimation != SCOPE_DECIMATE_MAX) {
        cfg->decimation = SCOPE_DECIMATE_MEAN;
    }
    if (!(cfg->y_max > cfg->y_min)) {
        cfg->y_min = 0.0f;
        cfg->y_max = 1000.0f;
    }

    /* The ring must hold one full window plus slack for the UI to lag. */
    while ((uint32_t)cfg->window_s * cfg->sample_rate_hz >
           SAMPLE_RING_CAPACITY - SAMPLE_RING_HEADROOM) {
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

/* Backup SRAM writes are plain stores with no wear, so every change is saved
 * at once; there is nothing to debounce. */
void settings_set(const settings_t *cfg)
{
    settings_t next = *cfg;
    bool locked;

    settings_validate(&next);
    locked = settings_lock();
    settings_current = next;
    settings_gen++;
    (void)settings_save(&next);
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

/* CRC-32 (IEEE, reflected) over whole words, least significant byte first. */
static uint32_t settings_crc(const uint32_t *words, uint32_t count)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0U; i < count; i++) {
        for (uint32_t byte = 0U; byte < 4U; byte++) {
            crc ^= (words[i] >> (8U * byte)) & 0xFFU;
            for (uint32_t bit = 0U; bit < 8U; bit++) {
                crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
            }
        }
    }
    return ~crc;
}

static uint32_t float_bits(float f)
{
    uint32_t w;
    memcpy(&w, &f, sizeof(w));
    return w;
}

static float bits_float(uint32_t w)
{
    float f;
    memcpy(&f, &w, sizeof(f));
    return f;
}

bool settings_load(settings_t *out)
{
    uint32_t w[SETTINGS_W_COUNT];

    for (uint32_t i = 0U; i < SETTINGS_W_COUNT; i++) {
        if (bk_ram_rd(&w[i], SETTINGS_BKRAM_BASE + i) != 0) {
            return false;
        }
    }
    if (w[SETTINGS_W_MAGIC] != SETTINGS_MAGIC ||
        w[SETTINGS_W_VERSION] != SETTINGS_VERSION ||
        w[SETTINGS_W_CRC] != settings_crc(w, SETTINGS_W_CRC)) {
        return false;
    }

    /* settings_init() validates, so out-of-range values are clamped there. */
    memset(out, 0, sizeof(*out));
    out->mode = (scope_mode_t)w[SETTINGS_W_MODE];
    out->window_s = (uint16_t)w[SETTINGS_W_WINDOW];
    out->sample_rate_hz = (uint16_t)w[SETTINGS_W_RATE];
    out->y_auto = (w[SETTINGS_W_Y_AUTO] != 0U);
    out->y_min = bits_float(w[SETTINGS_W_Y_MIN]);
    out->y_max = bits_float(w[SETTINGS_W_Y_MAX]);
    out->decimation = (scope_decimation_t)w[SETTINGS_W_DECIMATION];
    return true;
}

bool settings_save(const settings_t *cfg)
{
    uint32_t w[SETTINGS_W_COUNT];

    w[SETTINGS_W_MAGIC] = SETTINGS_MAGIC;
    w[SETTINGS_W_VERSION] = SETTINGS_VERSION;
    w[SETTINGS_W_MODE] = (uint32_t)cfg->mode;
    w[SETTINGS_W_WINDOW] = cfg->window_s;
    w[SETTINGS_W_RATE] = cfg->sample_rate_hz;
    w[SETTINGS_W_Y_AUTO] = cfg->y_auto ? 1U : 0U;
    w[SETTINGS_W_Y_MIN] = float_bits(cfg->y_min);
    w[SETTINGS_W_Y_MAX] = float_bits(cfg->y_max);
    w[SETTINGS_W_DECIMATION] = (uint32_t)cfg->decimation;
    w[SETTINGS_W_CRC] = settings_crc(w, SETTINGS_W_CRC);

    /* CRC last: until it lands, a reset reads the record back as invalid. */
    for (uint32_t i = 0U; i < SETTINGS_W_COUNT; i++) {
        if (bk_ram_wr(&w[i], SETTINGS_BKRAM_BASE + i) != 0) {
            return false;
        }
    }
    return true;
}
