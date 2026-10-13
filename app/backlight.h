#ifndef BACKLIGHT_H
#define BACKLIGHT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BACKLIGHT_FADE_MS (300U)

/* Start the backlight PWM at a fixed level (0..100 %). Call after
 * clock_init(); safe before the scheduler starts. */
void backlight_init(uint8_t percent);

/* Ramp linearly from the current level to percent over fade_ms (0 = jump).
 * A new call replaces a fade in progress, starting from where it got to. */
void backlight_fade_to(uint8_t percent, uint32_t fade_ms);

/* Level the backlight is at or fading towards. */
uint8_t backlight_target(void);

#ifdef __cplusplus
}
#endif

#endif
