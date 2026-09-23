#include "ui_settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensor_task.h"
#include "settings.h"
#include "ui.h"
#include "ui_style.h"

#define SETTINGS_HEADER_HEIGHT   48
#define SETTINGS_ROW_HEIGHT      56
#define SETTINGS_ROW_PAD         10
#define SETTINGS_CONTROL_WIDTH   170
#define SETTINGS_KEYBOARD_HEIGHT 220
#define SETTINGS_SAVE_DEBOUNCE_MS 2000U
#define SETTINGS_STATUS_PERIOD_MS 500U

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *list;
    lv_obj_t *mode_matrix;
    lv_obj_t *window_dd;
    lv_obj_t *rate_dd;
    lv_obj_t *filter_dd;
    lv_obj_t *yrange_dd;
    lv_obj_t *ymin_row;
    lv_obj_t *ymax_row;
    lv_obj_t *ymin_ta;
    lv_obj_t *ymax_ta;
    lv_obj_t *decimation_dd;
    lv_obj_t *refresh_dd;
    lv_obj_t *status_label;
    lv_obj_t *keyboard;
    lv_obj_t *msgbox;
    lv_timer_t *save_timer;
    lv_timer_t *status_timer;
    bool syncing;
} settings_ui_t;

static settings_ui_t u;
static const char *mode_map[] = {"mA", "mW", ""};

/*====================
 * Persistence debounce
 *====================*/
/* One-shot: fires 2 s after the last change, then pauses itself. */
static void save_timer_cb(lv_timer_t *timer)
{
    settings_t cfg;

    lv_timer_pause(timer);
    settings_get(&cfg);
    (void)settings_save(&cfg);
}

static void schedule_save(void)
{
    if (u.save_timer == NULL) {
        u.save_timer = lv_timer_create(save_timer_cb, SETTINGS_SAVE_DEBOUNCE_MS, NULL);
    }
    lv_timer_reset(u.save_timer);
    lv_timer_resume(u.save_timer);
}

/* Apply a modification through the settings store. */
static void commit(const settings_t *cfg)
{
    settings_set(cfg);
    schedule_save();
    ui_settings_sync();
}

/*====================
 * Control events
 *====================*/
static void mode_cb(lv_event_t *e)
{
    uint32_t id = lv_buttonmatrix_get_selected_button(lv_event_get_target_obj(e));
    settings_t cfg;

    if (u.syncing || id == LV_BUTTONMATRIX_BUTTON_NONE) {
        return;
    }
    settings_get(&cfg);
    cfg.mode = (id == 0U) ? SCOPE_MODE_MA : SCOPE_MODE_MW;
    commit(&cfg);
}

static void dropdown_cb(lv_event_t *e)
{
    lv_obj_t *dd = lv_event_get_target_obj(e);
    uint32_t sel = lv_dropdown_get_selected(dd);
    settings_t cfg;

    if (u.syncing) {
        return;
    }
    settings_get(&cfg);

    if (dd == u.window_dd && sel < SETTINGS_WINDOW_COUNT) {
        cfg.window_s = settings_window_options_s[sel];
    } else if (dd == u.rate_dd && sel < SETTINGS_RATE_COUNT) {
        cfg.sample_rate_hz = settings_rate_options_hz[sel];
    } else if (dd == u.filter_dd && sel < SETTINGS_FILTER_COUNT) {
        cfg.filter = (scope_filter_t)sel;
    } else if (dd == u.yrange_dd) {
        cfg.y_auto = (sel == 0U);
    } else if (dd == u.decimation_dd) {
        cfg.decimation = (sel == 1U) ? SCOPE_DECIMATE_MAX : SCOPE_DECIMATE_MEAN;
    } else if (dd == u.refresh_dd && sel < SETTINGS_REFRESH_COUNT) {
        cfg.refresh_hz = settings_refresh_options_hz[sel];
    }
    commit(&cfg);
}

static void apply_manual_range(void)
{
    settings_t cfg;
    float y_min = strtof(lv_textarea_get_text(u.ymin_ta), NULL);
    float y_max = strtof(lv_textarea_get_text(u.ymax_ta), NULL);

    settings_get(&cfg);
    if (y_max > y_min) {
        cfg.y_min = y_min;
        cfg.y_max = y_max;
    }
    commit(&cfg);   /* sync() rewrites the text if the input was rejected */
}

static void textarea_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target_obj(e);

    if (code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(u.keyboard, ta);
        lv_obj_remove_flag(u.keyboard, LV_OBJ_FLAG_HIDDEN);
        lv_obj_scroll_to_view(ta, LV_ANIM_ON);
    } else if (code == LV_EVENT_DEFOCUSED) {
        apply_manual_range();
    }
}

static void keyboard_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_obj_t *ta = lv_keyboard_get_textarea(u.keyboard);
        lv_obj_add_flag(u.keyboard, LV_OBJ_FLAG_HIDDEN);
        if (ta != NULL) {
            lv_obj_remove_state(ta, LV_STATE_FOCUSED);
            lv_keyboard_set_textarea(u.keyboard, NULL);
        }
        apply_manual_range();
    }
}

static void msgbox_button_cb(lv_event_t *e)
{
    bool restore = (lv_event_get_user_data(e) != NULL);

    if (u.msgbox != NULL) {
        lv_msgbox_close(u.msgbox);
        u.msgbox = NULL;
    }
    if (restore) {
        settings_reset_defaults();
        schedule_save();
        ui_settings_sync();
    }
}

static void restore_cb(lv_event_t *e)
{
    lv_obj_t *button;
    (void)e;

    if (u.msgbox != NULL) {
        return;
    }
    u.msgbox = lv_msgbox_create(NULL);
    lv_msgbox_add_title(u.msgbox, "Restore defaults");
    lv_msgbox_add_text(u.msgbox, "Reset every setting to its default value?");
    button = lv_msgbox_add_footer_button(u.msgbox, "Cancel");
    lv_obj_add_event_cb(button, msgbox_button_cb, LV_EVENT_CLICKED, NULL);
    button = lv_msgbox_add_footer_button(u.msgbox, "Restore");
    lv_obj_add_event_cb(button, msgbox_button_cb, LV_EVENT_CLICKED, (void *)1);
}

static void back_cb(lv_event_t *e)
{
    (void)e;
    ui_show_scope();
}

static void gesture_cb(lv_event_t *e)
{
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
    (void)e;
    if (dir == LV_DIR_RIGHT) {
        ui_show_scope();
    }
}

static void status_timer_cb(lv_timer_t *timer)
{
    sensor_stats_t st;
    (void)timer;

    if (lv_screen_active() != u.screen) {
        return;
    }
    sensor_task_get_stats(&st);
    lv_label_set_text_fmt(u.status_label,
                          "%s%lu Hz achieved (period %lu ms)\n"
                          "samples %lu   dropped %lu   errors %lu",
                          st.demo ? "DEMO DATA  " : "",
                          (unsigned long)st.achieved_hz,
                          (unsigned long)st.period_ms,
                          (unsigned long)st.samples,
                          (unsigned long)st.dropped,
                          (unsigned long)st.errors);
}

/*====================
 * Construction helpers
 *====================*/
static lv_obj_t *create_row(const char *title)
{
    lv_obj_t *row = lv_obj_create(u.list);
    lv_obj_t *label;

    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), SETTINGS_ROW_HEIGHT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_hor(row, SETTINGS_ROW_PAD, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);

    label = ui_style_create_label(row, title, UI_FONT_BODY, UI_COLOR_TEXT);
    (void)label;
    return row;
}

static lv_obj_t *create_dropdown(lv_obj_t *row, const char *options)
{
    lv_obj_t *dd = lv_dropdown_create(row);
    lv_dropdown_set_options(dd, options);
    lv_obj_set_width(dd, SETTINGS_CONTROL_WIDTH);
    lv_obj_set_style_bg_color(dd, lv_color_hex(UI_COLOR_PANEL_ALT), 0);
    lv_obj_set_style_border_color(dd, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_text_font(dd, UI_FONT_BODY, 0);
    lv_obj_add_event_cb(dd, dropdown_cb, LV_EVENT_VALUE_CHANGED, NULL);
    return dd;
}

static lv_obj_t *create_textarea(lv_obj_t *row)
{
    lv_obj_t *ta = lv_textarea_create(row);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_accepted_chars(ta, "0123456789.-");
    lv_textarea_set_max_length(ta, 10);
    lv_obj_set_width(ta, SETTINGS_CONTROL_WIDTH);
    lv_obj_set_style_bg_color(ta, lv_color_hex(UI_COLOR_PANEL_ALT), 0);
    lv_obj_set_style_border_color(ta, lv_color_hex(UI_COLOR_BORDER), 0);
    lv_obj_set_style_text_font(ta, UI_FONT_BODY, 0);
    lv_obj_add_event_cb(ta, textarea_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta, textarea_cb, LV_EVENT_DEFOCUSED, NULL);
    return ta;
}

static void build_header(void)
{
    lv_obj_t *header = ui_style_create_panel(u.screen);
    lv_obj_t *back;
    lv_obj_t *title;

    lv_obj_set_size(header, LV_PCT(100), SETTINGS_HEADER_HEIGHT);
    lv_obj_set_pos(header, 0, 0);

    back = ui_style_create_button(header, LV_SYMBOL_LEFT " Back", NULL);
    lv_obj_align(back, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_add_event_cb(back, back_cb, LV_EVENT_CLICKED, NULL);

    title = ui_style_create_label(header, "SETTINGS", UI_FONT_TITLE, UI_COLOR_TEXT);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);
}

static void build_list(void)
{
    lv_obj_t *row;
    lv_obj_t *button;

    u.list = lv_obj_create(u.screen);
    lv_obj_remove_style_all(u.list);
    lv_obj_set_size(u.list, LV_PCT(100), LV_VER_RES - SETTINGS_HEADER_HEIGHT);
    lv_obj_set_pos(u.list, 0, SETTINGS_HEADER_HEIGHT);
    lv_obj_set_flex_flow(u.list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(u.list, LV_DIR_VER);
    lv_obj_set_style_pad_bottom(u.list, SETTINGS_KEYBOARD_HEIGHT, 0);

    row = create_row("Display mode");
    u.mode_matrix = lv_buttonmatrix_create(row);
    lv_buttonmatrix_set_map(u.mode_matrix, mode_map);
    lv_buttonmatrix_set_button_ctrl(u.mode_matrix, 0, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_button_ctrl(u.mode_matrix, 1, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(u.mode_matrix, true);
    lv_obj_set_size(u.mode_matrix, SETTINGS_CONTROL_WIDTH, 40);
    lv_obj_set_style_pad_all(u.mode_matrix, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(u.mode_matrix, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(u.mode_matrix, lv_color_hex(UI_COLOR_PANEL_ALT), LV_PART_MAIN);
    lv_obj_set_style_border_color(u.mode_matrix, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_bg_color(u.mode_matrix, lv_color_hex(UI_COLOR_PANEL_ALT), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(u.mode_matrix, lv_color_hex(UI_COLOR_ACCENT),
                              LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(u.mode_matrix, lv_color_hex(UI_COLOR_BG),
                                LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_add_event_cb(u.mode_matrix, mode_cb, LV_EVENT_VALUE_CHANGED, NULL);

    row = create_row("Time window");
    u.window_dd = create_dropdown(row, "1 s\n2 s\n5 s\n10 s\n30 s\n60 s");

    row = create_row("Sample rate");
    u.rate_dd = create_dropdown(row, "10 Hz\n50 Hz\n100 Hz\n200 Hz\n500 Hz");

    row = create_row("Filter");
    u.filter_dd = create_dropdown(row, "Fast\nNormal\nSmooth\nUltra smooth");

    row = create_row("Y range");
    u.yrange_dd = create_dropdown(row, "Auto\nManual");

    u.ymin_row = create_row("Y min");
    u.ymin_ta = create_textarea(u.ymin_row);

    u.ymax_row = create_row("Y max");
    u.ymax_ta = create_textarea(u.ymax_row);

    row = create_row("Decimation");
    u.decimation_dd = create_dropdown(row, "Mean\nMax");

    row = create_row("Chart refresh");
    u.refresh_dd = create_dropdown(row, "15 Hz\n30 Hz\n60 Hz");

    row = create_row("Sensor");
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_ver(row, SETTINGS_ROW_PAD, 0);
    u.status_label = ui_style_create_label(row, "--", UI_FONT_SMALL, UI_COLOR_TEXT_DIM);
    lv_obj_set_style_text_align(u.status_label, LV_TEXT_ALIGN_RIGHT, 0);

    row = create_row("");
    lv_obj_set_style_border_width(row, 0, 0);
    button = ui_style_create_button(row, "Restore defaults", NULL);
    lv_obj_add_event_cb(button, restore_cb, LV_EVENT_CLICKED, NULL);
}

static void build_keyboard(void)
{
    u.keyboard = lv_keyboard_create(u.screen);
    lv_keyboard_set_mode(u.keyboard, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_size(u.keyboard, LV_PCT(100), SETTINGS_KEYBOARD_HEIGHT);
    lv_obj_align(u.keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_flag(u.keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(u.keyboard, keyboard_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(u.keyboard, keyboard_cb, LV_EVENT_CANCEL, NULL);
}

/*====================
 * Public
 *====================*/
void ui_settings_sync(void)
{
    settings_t cfg;
    char buf[16];

    settings_get(&cfg);
    u.syncing = true;

    lv_buttonmatrix_set_button_ctrl(u.mode_matrix, (uint32_t)cfg.mode,
                                    LV_BUTTONMATRIX_CTRL_CHECKED);
    lv_dropdown_set_selected(u.window_dd, settings_window_index(cfg.window_s));
    lv_dropdown_set_selected(u.rate_dd, settings_rate_index(cfg.sample_rate_hz));
    lv_dropdown_set_selected(u.filter_dd, (uint32_t)cfg.filter);
    lv_dropdown_set_selected(u.yrange_dd, cfg.y_auto ? 0U : 1U);
    lv_dropdown_set_selected(u.decimation_dd,
                             (cfg.decimation == SCOPE_DECIMATE_MAX) ? 1U : 0U);
    lv_dropdown_set_selected(u.refresh_dd, settings_refresh_index(cfg.refresh_hz));

    snprintf(buf, sizeof(buf), "%g", (double)cfg.y_min);
    lv_textarea_set_text(u.ymin_ta, buf);
    snprintf(buf, sizeof(buf), "%g", (double)cfg.y_max);
    lv_textarea_set_text(u.ymax_ta, buf);

    if (cfg.y_auto) {
        lv_obj_add_flag(u.ymin_row, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(u.ymax_row, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(u.ymin_row, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(u.ymax_row, LV_OBJ_FLAG_HIDDEN);
    }

    u.syncing = false;
}

lv_obj_t *ui_settings_create(void)
{
    memset(&u, 0, sizeof(u));

    u.screen = lv_obj_create(NULL);
    lv_obj_add_style(u.screen, &ui_style_screen, 0);
    lv_obj_remove_flag(u.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(u.screen, gesture_cb, LV_EVENT_GESTURE, NULL);

    build_header();
    build_list();
    build_keyboard();
    ui_settings_sync();

    u.status_timer = lv_timer_create(status_timer_cb, SETTINGS_STATUS_PERIOD_MS, NULL);
    return u.screen;
}
