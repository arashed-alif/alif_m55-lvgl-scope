#include "ui.h"

#include "lvgl.h"

#include "ui_scope.h"
#include "ui_settings.h"
#include "ui_style.h"

#define UI_SCREEN_ANIM_MS 200U

static lv_obj_t *scope_screen;
static lv_obj_t *settings_screen;

void ui_init(void)
{
    lv_display_t *disp = lv_display_get_default();
    lv_theme_t *theme = lv_theme_default_init(disp,
                                              lv_color_hex(UI_COLOR_ACCENT),
                                              lv_color_hex(UI_COLOR_CURSOR),
                                              true, UI_FONT_BODY);
    lv_display_set_theme(disp, theme);

    ui_style_init();
    scope_screen = ui_scope_create();
    settings_screen = ui_settings_create();

    lv_screen_load(scope_screen);
}

void ui_show_scope(void)
{
    if (lv_screen_active() == scope_screen) {
        return;
    }
    lv_screen_load_anim(scope_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT,
                        UI_SCREEN_ANIM_MS, 0, false);
}

void ui_show_settings(void)
{
    if (lv_screen_active() == settings_screen) {
        return;
    }
    ui_settings_sync();
    lv_screen_load_anim(settings_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT,
                        UI_SCREEN_ANIM_MS, 0, false);
}
