#ifndef UI_H
#define UI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Build both screens and load the scope screen. Call from the LVGL task
 * after lv_port_disp_init(). */
void ui_init(void);

void ui_show_scope(void);
void ui_show_settings(void);

#ifdef __cplusplus
}
#endif

#endif
