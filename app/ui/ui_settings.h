#ifndef UI_SETTINGS_H
#define UI_SETTINGS_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Build the settings screen. Returns the screen object. */
lv_obj_t *ui_settings_create(void);

/* Push the current settings_t into the controls (call before showing). */
void ui_settings_sync(void);

#ifdef __cplusplus
}
#endif

#endif
