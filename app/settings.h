#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

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
} settings_t;

#define SETTINGS_WINDOW_COUNT 6U
#define SETTINGS_RATE_COUNT 3U

extern const uint16_t settings_window_options_s[SETTINGS_WINDOW_COUNT];
extern const uint16_t settings_rate_options_hz[SETTINGS_RATE_COUNT];

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

/* Backup SRAM persistence (see settings.c). load returns false when no valid
 * record is stored. */
bool settings_load(settings_t *out);
bool settings_save(const settings_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
