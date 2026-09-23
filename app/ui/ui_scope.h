#ifndef UI_SCOPE_H
#define UI_SCOPE_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Build the scope screen and start its update timer. Returns the screen. */
lv_obj_t *ui_scope_create(void);

#ifdef __cplusplus
}
#endif

#endif
