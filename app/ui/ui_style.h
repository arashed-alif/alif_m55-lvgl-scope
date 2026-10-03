#ifndef UI_STYLE_H
#define UI_STYLE_H

#include <stdint.h>

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UI_COLOR_BG        0x0F141AU
#define UI_COLOR_PANEL     0x18222BU
#define UI_COLOR_PANEL_ALT 0x1F2C36U
#define UI_COLOR_CHART_BG  0x0A0F14U
#define UI_COLOR_GRID      0x223038U
#define UI_COLOR_BORDER    0x2E3E48U
#define UI_COLOR_TEXT      0xE8EEF2U
#define UI_COLOR_TEXT_DIM  0x8FA3AEU
#define UI_COLOR_ACCENT    0xF2B134U
#define UI_COLOR_RUN       0x62C370U
#define UI_COLOR_HOLD      0xE8873AU
#define UI_COLOR_CURSOR    0x5EC8F2U
#define UI_COLOR_ERROR     0xE05A5AU

#define UI_FONT_SMALL   (&lv_font_montserrat_14)
#define UI_FONT_BODY    (&lv_font_montserrat_16)
#define UI_FONT_CONTROL (&lv_font_montserrat_20)   /* buttons, dropdowns, settings rows */
#define UI_FONT_TITLE   (&lv_font_montserrat_24)
#define UI_FONT_READOUT (&lv_font_montserrat_28)

/* Touch target sizes for the 480x800 portrait panel: taller rather than wider.
 * UI_CONTROL_PAD_VER makes dropdowns and text fields 50 px: a 22 px
 * UI_FONT_CONTROL line plus padding and the theme's 2 px border, top and
 * bottom. */
#define UI_BUTTON_HEIGHT   55
#define UI_BUTTON_PAD_HOR  20
#define UI_CONTROL_PAD_VER 12
#define UI_SELECTOR_HEIGHT 60   /* mA/mW button matrix */

extern lv_style_t ui_style_screen;
extern lv_style_t ui_style_panel;
extern lv_style_t ui_style_button;
extern lv_style_t ui_style_button_checked;
/* Dropdowns and text fields, on top of the theme. */
extern lv_style_t ui_style_control;

void ui_style_init(void);

/* Flat, borderless container with the panel colour. */
lv_obj_t *ui_style_create_panel(lv_obj_t *parent);
lv_obj_t *ui_style_create_label(lv_obj_t *parent, const char *text,
                                const lv_font_t *font, uint32_t color_hex);
/* Button with a centred text label; returns the button, label via out param. */
lv_obj_t *ui_style_create_button(lv_obj_t *parent, const char *text,
                                 lv_obj_t **label_out);

#ifdef __cplusplus
}
#endif

#endif
