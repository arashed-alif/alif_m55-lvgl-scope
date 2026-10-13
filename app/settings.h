#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#include "sample_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCOPE_MODE_MA = 0,
    SCOPE_MODE_MW = 1
} scope_mode_t;

typedef enum {
    SCOPE_DECIMATE_MEAN = 0,
    SCOPE_DECIMATE_MAX = 1
} scope_decimation_t;

typedef struct {
    scope_mode_t mode;
    uint16_t window_s;          /* one of settings_window_options_s */
    uint16_t sample_rate_hz;    /* one of settings_rate_options_hz; also sets
                                 * the INA228 averaging (see sensor_task.c) */
    bool y_auto;
    float y_min;                /* manual range, display units (mA or mW) */
    float y_max;
    scope_decimation_t decimation;
    bool auto_dim;              /* dim when set down, brighten when picked up */
    uint16_t bright_pct;        /* one of settings_bright_options_pct */
    uint16_t dim_pct;           /* one of settings_dim_options_pct; 0 = off */
    uint16_t dim_after_s;       /* one of settings_dim_after_options_s */
    bool ch_enabled[SAMPLE_CHANNELS];   /* INA228 channels sampled and drawn */
} settings_t;

#define SETTINGS_WINDOW_COUNT 6U
#define SETTINGS_RATE_COUNT 3U
#define SETTINGS_BRIGHT_COUNT 4U
#define SETTINGS_DIM_COUNT 5U
#define SETTINGS_DIM_AFTER_COUNT 5U

extern const uint16_t settings_window_options_s[SETTINGS_WINDOW_COUNT];
extern const uint16_t settings_rate_options_hz[SETTINGS_RATE_COUNT];
extern const uint16_t settings_bright_options_pct[SETTINGS_BRIGHT_COUNT];
extern const uint16_t settings_dim_options_pct[SETTINGS_DIM_COUNT];
extern const uint16_t settings_dim_after_options_s[SETTINGS_DIM_AFTER_COUNT];

/* Create the mutex and load defaults (then persisted values, if any). */
void settings_init(void);
void settings_defaults(settings_t *out);
/* Clamp every field to a legal value. */
void settings_validate(settings_t *cfg);

/* Thread-safe copy in / copy out. settings_set() bumps the generation and
 * saves to backup SRAM. */
void settings_get(settings_t *out);
void settings_set(const settings_t *cfg);
void settings_reset_defaults(void);
uint32_t settings_generation(void);

/* Index helpers for UI dropdowns. */
uint32_t settings_window_index(uint16_t window_s);
uint32_t settings_rate_index(uint16_t rate_hz);
uint32_t settings_bright_index(uint16_t pct);
uint32_t settings_dim_index(uint16_t pct);
uint32_t settings_dim_after_index(uint16_t s);

/* Backup SRAM persistence (see settings.c). load returns false when no valid
 * record is stored. */
bool settings_load(settings_t *out);
bool settings_save(const settings_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
