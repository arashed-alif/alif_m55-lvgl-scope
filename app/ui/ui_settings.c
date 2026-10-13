#include "ui_settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_task.h"
#include "sensor_task.h"
#include "settings.h"
#include "ui.h"
#include "ui_style.h"

#define SETTINGS_HEADER_HEIGHT   60
#define SETTINGS_ROW_HEIGHT      64
#define SETTINGS_ROW_PAD         10
#define SETTINGS_CONTROL_WIDTH   187
#define SETTINGS_KEYBOARD_HEIGHT 220
#define SETTINGS_STATUS_PERIOD_MS 500U
#define SETTINGS_OPTIONS_LEN     64     /* dropdown option text built at run time */
/* Restore-defaults dialog: box and title bar at 120 % of the LVGL msgbox
 * defaults (DPI*2 wide, DPI/3 title bar); buttons as tall as the other
 * buttons, with 5 px more side padding than the theme's 20 px. */
#define SETTINGS_MSGBOX_WIDTH          (LV_DPI_DEF * 2 * 6 / 5)
#define SETTINGS_MSGBOX_TITLE_HEIGHT   (LV_DPI_DEF / 3 * 6 / 5)
#define SETTINGS_MSGBOX_BUTTON_PAD_HOR 25

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *list;
    /* Per channel: a Show/Hide dropdown while the INA228 is present, a
     * "not found" label while it isn't. */
    lv_obj_t *ch_dd[SAMPLE_CHANNELS];
    lv_obj_t *ch_absent[SAMPLE_CHANNELS];
    int8_t ch_present_shown[SAMPLE_CHANNELS];   /* -1 unknown, else 0/1 */
    lv_obj_t *window_dd;
    lv_obj_t *rate_dd;
    lv_obj_t *yrange_dd;
    lv_obj_t *ymin_row;
    lv_obj_t *ymax_row;
    lv_obj_t *ymin_ta;
    lv_obj_t *ymax_ta;
    lv_obj_t *decimation_dd;
    lv_obj_t *bright_dd;
    lv_obj_t *autodim_dd;
    lv_obj_t *dim_row;
    lv_obj_t *dim_dd;
    lv_obj_t *dim_after_row;
    lv_obj_t *dim_after_dd;
    lv_obj_t *status_label;
    lv_obj_t *keyboard;
    lv_obj_t *msgbox;
    lv_timer_t *status_timer;
    lv_style_t row_style;
    bool syncing;
} settings_ui_t;

static settings_ui_t u;

/* Apply a modification through the settings store, which also saves it. */
static void commit(const settings_t *cfg)
{
    settings_set(cfg);
    ui_settings_sync();
}

/*====================
 * Control events
 *====================*/
static void dropdown_cb(lv_event_t *e)
{
    lv_obj_t *dd = lv_event_get_target_obj(e);
    uint32_t sel = lv_dropdown_get_selected(dd);
    settings_t cfg;

    if (u.syncing) {
        return;
    }
    settings_get(&cfg);

    for (uint32_t i = 0U; i < SAMPLE_CHANNELS; i++) {
        if (dd == u.ch_dd[i]) {
            cfg.ch_enabled[i] = (sel == 0U);
        }
    }
    if (dd == u.window_dd && sel < SETTINGS_WINDOW_COUNT) {
        cfg.window_s = settings_window_options_s[sel];
    } else if (dd == u.rate_dd && sel < SETTINGS_RATE_COUNT) {
        cfg.sample_rate_hz = settings_rate_options_hz[sel];
    } else if (dd == u.yrange_dd) {
        cfg.y_auto = (sel == 0U);
    } else if (dd == u.decimation_dd) {
        cfg.decimation = (sel == 1U) ? SCOPE_DECIMATE_MAX : SCOPE_DECIMATE_MEAN;
    } else if (dd == u.bright_dd && sel < SETTINGS_BRIGHT_COUNT) {
        cfg.bright_pct = settings_bright_options_pct[sel];
    } else if (dd == u.autodim_dd) {
        cfg.auto_dim = (sel == 0U);
    } else if (dd == u.dim_dd && sel < SETTINGS_DIM_COUNT) {
        cfg.dim_pct = settings_dim_options_pct[sel];
    } else if (dd == u.dim_after_dd && sel < SETTINGS_DIM_AFTER_COUNT) {
        cfg.dim_after_s = settings_dim_after_options_s[sel];
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

/* The keyboard is open exactly while it is bound to a textarea. */
static void show_keyboard(lv_obj_t *ta)
{
    int32_t list_h = LV_VER_RES - SETTINGS_HEADER_HEIGHT - SETTINGS_KEYBOARD_HEIGHT;

    if (lv_keyboard_get_textarea(u.keyboard) == ta) {
        return;
    }
    lv_keyboard_set_textarea(u.keyboard, ta);
    lv_obj_remove_flag(u.keyboard, LV_OBJ_FLAG_HIDDEN);

    /* Scrolling can't reveal a row the keyboard covers while it is still inside
     * the list's viewport, so shrink the viewport to the space above it. */
    if (list_h < SETTINGS_ROW_HEIGHT) {
        list_h = SETTINGS_ROW_HEIGHT;
    }
    lv_obj_set_height(u.list, list_h);
    lv_obj_update_layout(u.list);
    /* scroll_to_view only scrolls the direct parent; for the textarea that is
     * its row, which never needs scrolling, so bring the row into view. */
    lv_obj_scroll_to_view(lv_obj_get_parent(ta), LV_ANIM_OFF);
}

/* Returns true if the keyboard was open, i.e. an edit was in progress. */
static bool hide_keyboard(void)
{
    if (lv_keyboard_get_textarea(u.keyboard) == NULL) {
        return false;
    }
    lv_keyboard_set_textarea(u.keyboard, NULL);     /* also clears the textarea's FOCUSED state */
    lv_obj_add_flag(u.keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_height(u.list, LV_VER_RES - SETTINGS_HEADER_HEIGHT);
    return true;
}

static void textarea_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target_obj(e);

    /* Re-tapping the textarea that still holds indev focus only sends CLICKED. */
    if (code == LV_EVENT_FOCUSED || code == LV_EVENT_CLICKED) {
        show_keyboard(ta);
    } else if (code == LV_EVENT_DEFOCUSED) {
        /* A late DEFOCUSED after READY/CANCEL finds the keyboard unbound: the
         * edit was already committed or reverted. */
        if (lv_keyboard_get_textarea(u.keyboard) == ta) {
            (void)hide_keyboard();
            apply_manual_range();
        }
    }
}

static void keyboard_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_READY) {
        if (hide_keyboard()) {
            apply_manual_range();
        }
    } else if (code == LV_EVENT_CANCEL) {
        if (hide_keyboard()) {
            ui_settings_sync();     /* revert the text to the stored values */
        }
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
    lv_obj_set_style_text_font(u.msgbox, UI_FONT_CONTROL, 0);    /* title, text and buttons */
    lv_msgbox_add_title(u.msgbox, "Restore defaults");
    lv_msgbox_add_text(u.msgbox, "Reset every setting to its default value?");
    button = lv_msgbox_add_footer_button(u.msgbox, "Cancel");
    lv_obj_set_height(button, UI_BUTTON_HEIGHT);
    lv_obj_set_style_pad_hor(button, SETTINGS_MSGBOX_BUTTON_PAD_HOR, 0);
    lv_obj_add_event_cb(button, msgbox_button_cb, LV_EVENT_CLICKED, NULL);
    button = lv_msgbox_add_footer_button(u.msgbox, "Restore");
    lv_obj_set_height(button, UI_BUTTON_HEIGHT);
    lv_obj_set_style_pad_hor(button, SETTINGS_MSGBOX_BUTTON_PAD_HOR, 0);
    lv_obj_add_event_cb(button, msgbox_button_cb, LV_EVENT_CLICKED, (void *)1);

    /* LVGL sizes the box and its bars from LV_DPI_DEF, with footer buttons at
     * 100 % of a fixed-height footer. Give the buttons a real height instead
     * and let the footer wrap them (plus its theme padding). */
    lv_obj_set_width(u.msgbox, SETTINGS_MSGBOX_WIDTH);
    lv_obj_set_height(lv_msgbox_get_header(u.msgbox), SETTINGS_MSGBOX_TITLE_HEIGHT);
    lv_obj_set_height(lv_msgbox_get_footer(u.msgbox), LV_SIZE_CONTENT);
}

static void leave_screen(void)
{
    if (hide_keyboard()) {
        apply_manual_range();
    }
    ui_show_scope();
}

static void back_cb(lv_event_t *e)
{
    (void)e;
    leave_screen();
}

static void gesture_cb(lv_event_t *e)
{
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
    (void)e;
    if (dir == LV_DIR_RIGHT) {
        leave_screen();
    }
}

/* Swap each channel's control for "not found" when its INA228 didn't answer.
 * Presence is only known once the sensor task has probed, so this runs from
 * the status timer as well as from sync. */
static void sync_channel_presence(void)
{
    for (uint32_t i = 0U; i < SAMPLE_CHANNELS; i++) {
        int8_t present = sensor_task_channel_present(i) ? 1 : 0;

        if (present == u.ch_present_shown[i]) {
            continue;
        }
        u.ch_present_shown[i] = present;
        if (present) {
            lv_obj_remove_flag(u.ch_dd[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(u.ch_absent[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(u.ch_dd[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_remove_flag(u.ch_absent[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void status_timer_cb(lv_timer_t *timer)
{
    sensor_stats_t st;
    motion_status_t ms;
    (void)timer;

    if (lv_screen_active() != u.screen) {
        return;
    }
    sync_channel_presence();
    sensor_task_get_stats(&st);
    motion_task_get_status(&ms);
    lv_label_set_text_fmt(u.status_label,
                          "%s%lu Hz achieved (period %lu ms)\n"
                          "samples %lu   dropped %lu   errors %lu\n"
                          "IMU %s   %s   errors %lu",
                          st.demo ? "DEMO DATA  " : "",
                          (unsigned long)st.achieved_hz,
                          (unsigned long)st.period_ms,
                          (unsigned long)st.samples,
                          (unsigned long)st.dropped,
                          (unsigned long)st.errors,
                          ms.imu_ok ? "ok" : "not found",
                          ms.still ? "still" : "moving",
                          (unsigned long)ms.imu_errors);
}

/*====================
 * Construction helpers
 *====================*/
static lv_obj_t *create_row(const char *title)
{
    lv_obj_t *row = lv_obj_create(u.list);

    lv_obj_remove_style_all(row);
    lv_obj_add_style(row, &u.row_style, 0);
    lv_obj_set_size(row, LV_PCT(100), SETTINGS_ROW_HEIGHT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    (void)ui_style_create_label(row, title, UI_FONT_CONTROL, UI_COLOR_TEXT);
    return row;
}

static lv_obj_t *create_dropdown(lv_obj_t *row, const char *options)
{
    lv_obj_t *dd = lv_dropdown_create(row);
    lv_dropdown_set_options(dd, options);   /* copies the text */
    lv_obj_set_width(dd, SETTINGS_CONTROL_WIDTH);
    lv_obj_add_style(dd, &ui_style_control, 0);
    /* The open list: 22 px lines plus the theme's 16 px line spacing give 38 px
     * per option, and the 6-option list still fits the theme's 260 px limit. */
    lv_obj_set_style_text_font(lv_dropdown_get_list(dd), UI_FONT_CONTROL, 0);
    lv_obj_add_event_cb(dd, dropdown_cb, LV_EVENT_VALUE_CHANGED, NULL);
    return dd;
}

/* "<value> <unit>" per option, from the settings tables so the dropdown text
 * and the values it selects can't drift apart. */
static void format_options(char *buf, size_t len, const uint16_t *values,
                           uint32_t count, const char *unit)
{
    size_t pos = 0U;

    buf[0] = '\0';
    for (uint32_t i = 0U; i < count && pos < len; i++) {
        int n = snprintf(buf + pos, len - pos, "%s%u %s", (i == 0U) ? "" : "\n",
                         (unsigned int)values[i], unit);
        if (n < 0) {
            break;
        }
        pos += (size_t)n;
    }
}

/* Dim level options: "Off" for 0 %, else "<n> %". */
static void format_dim_options(char *buf, size_t len)
{
    size_t pos = 0U;

    buf[0] = '\0';
    for (uint32_t i = 0U; i < SETTINGS_DIM_COUNT && pos < len; i++) {
        unsigned int pct = settings_dim_options_pct[i];
        const char *sep = (i == 0U) ? "" : "\n";
        int n = (pct == 0U) ? snprintf(buf + pos, len - pos, "%sOff", sep)
                            : snprintf(buf + pos, len - pos, "%s%u %%", sep, pct);
        if (n < 0) {
            break;
        }
        pos += (size_t)n;
    }
}

static lv_obj_t *create_textarea(lv_obj_t *row)
{
    lv_obj_t *ta = lv_textarea_create(row);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_accepted_chars(ta, "0123456789.-");
    lv_textarea_set_max_length(ta, 10);
    lv_obj_set_width(ta, SETTINGS_CONTROL_WIDTH);
    lv_obj_add_style(ta, &ui_style_control, 0);
    lv_obj_add_event_cb(ta, textarea_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta, textarea_cb, LV_EVENT_CLICKED, NULL);
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
    char options[SETTINGS_OPTIONS_LEN];

    /* Shared by every row: a divider line along the bottom. */
    lv_style_init(&u.row_style);
    lv_style_set_pad_hor(&u.row_style, SETTINGS_ROW_PAD);
    lv_style_set_border_color(&u.row_style, lv_color_hex(UI_COLOR_BORDER));
    lv_style_set_border_width(&u.row_style, 1);
    lv_style_set_border_side(&u.row_style, LV_BORDER_SIDE_BOTTOM);

    u.list = lv_obj_create(u.screen);
    lv_obj_remove_style_all(u.list);
    lv_obj_set_size(u.list, LV_PCT(100), LV_VER_RES - SETTINGS_HEADER_HEIGHT);
    lv_obj_set_pos(u.list, 0, SETTINGS_HEADER_HEIGHT);
    lv_obj_set_flex_flow(u.list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(u.list, LV_DIR_VER);

    /* Channels first, as on a scope: CHn (I2C address) in its trace colour. */
    for (uint32_t i = 0U; i < SAMPLE_CHANNELS; i++) {
        char title[24];

        snprintf(title, sizeof(title), "CH%lu  (0x%02X)", (unsigned long)(i + 1U),
                 (unsigned int)sensor_task_channel_addr(i));
        row = create_row(title);
        lv_obj_set_style_text_color(lv_obj_get_child(row, 0),
                                    lv_color_hex(ui_style_channel_color(i)), 0);
        u.ch_dd[i] = create_dropdown(row, "Show\nHide");
        u.ch_absent[i] = ui_style_create_label(row, "not found", UI_FONT_CONTROL,
                                               UI_COLOR_TEXT_DIM);
        lv_obj_add_flag(u.ch_absent[i], LV_OBJ_FLAG_HIDDEN);
        u.ch_present_shown[i] = -1;
    }

    row = create_row("Time window");
    format_options(options, sizeof(options), settings_window_options_s,
                   SETTINGS_WINDOW_COUNT, "s");
    u.window_dd = create_dropdown(row, options);

    row = create_row("Sample rate");
    format_options(options, sizeof(options), settings_rate_options_hz,
                   SETTINGS_RATE_COUNT, "Hz");
    u.rate_dd = create_dropdown(row, options);

    row = create_row("Y range");
    u.yrange_dd = create_dropdown(row, "Auto\nManual");

    u.ymin_row = create_row("Y min");
    u.ymin_ta = create_textarea(u.ymin_row);

    u.ymax_row = create_row("Y max");
    u.ymax_ta = create_textarea(u.ymax_row);

    row = create_row("Decimation");
    u.decimation_dd = create_dropdown(row, "Mean\nMax");

    row = create_row("Brightness");
    format_options(options, sizeof(options), settings_bright_options_pct,
                   SETTINGS_BRIGHT_COUNT, "%");
    u.bright_dd = create_dropdown(row, options);

    row = create_row("Auto dim");
    u.autodim_dd = create_dropdown(row, "On\nOff");

    u.dim_row = create_row("Dim level");
    format_dim_options(options, sizeof(options));
    u.dim_dd = create_dropdown(u.dim_row, options);

    u.dim_after_row = create_row("Dim after");
    format_options(options, sizeof(options), settings_dim_after_options_s,
                   SETTINGS_DIM_AFTER_COUNT, "s");
    u.dim_after_dd = create_dropdown(u.dim_after_row, options);

    row = create_row("Sensor");
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_ver(row, SETTINGS_ROW_PAD, 0);
    u.status_label = ui_style_create_label(row, "--", UI_FONT_BODY, UI_COLOR_TEXT_DIM);
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
    lv_obj_set_style_text_font(u.keyboard, UI_FONT_CONTROL, 0);
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

    for (uint32_t i = 0U; i < SAMPLE_CHANNELS; i++) {
        lv_dropdown_set_selected(u.ch_dd[i], cfg.ch_enabled[i] ? 0U : 1U);
    }
    sync_channel_presence();

    lv_dropdown_set_selected(u.window_dd, settings_window_index(cfg.window_s));
    lv_dropdown_set_selected(u.rate_dd, settings_rate_index(cfg.sample_rate_hz));
    lv_dropdown_set_selected(u.yrange_dd, cfg.y_auto ? 0U : 1U);
    lv_dropdown_set_selected(u.decimation_dd,
                             (cfg.decimation == SCOPE_DECIMATE_MAX) ? 1U : 0U);
    lv_dropdown_set_selected(u.bright_dd, settings_bright_index(cfg.bright_pct));
    lv_dropdown_set_selected(u.autodim_dd, cfg.auto_dim ? 0U : 1U);
    lv_dropdown_set_selected(u.dim_dd, settings_dim_index(cfg.dim_pct));
    lv_dropdown_set_selected(u.dim_after_dd, settings_dim_after_index(cfg.dim_after_s));

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

    if (cfg.auto_dim) {
        lv_obj_remove_flag(u.dim_row, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(u.dim_after_row, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(u.dim_row, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(u.dim_after_row, LV_OBJ_FLAG_HIDDEN);
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
