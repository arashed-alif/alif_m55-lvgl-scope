#include "ui_scope.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sample_ring.h"
#include "sensor_task.h"
#include "settings.h"
#include "ui.h"
#include "ui_style.h"

/*====================
 * Layout / tuning
 *====================*/
#define SCOPE_HEADER_HEIGHT      48
/* Two footer rows on narrow (portrait) panels, one on wide ones. */
#define SCOPE_FOOTER_HEIGHT      ((LV_HOR_RES < 640) ? 96 : 52)
#define SCOPE_SCALE_WIDTH        62
#define SCOPE_MARGIN             6
#define SCOPE_CHART_MAX_POINTS   800
#define SCOPE_CHART_LINE_WIDTH   2
#define SCOPE_CHART_HDIV         9      /* 8 horizontal bands */
#define SCOPE_CHART_VDIV         11     /* 10 vertical bands (s/div) */
#define SCOPE_X_DIVISIONS        10
#define SCOPE_SCALE_TICKS        9
#define SCOPE_SCALE_MAJOR_EVERY  2
#define SCOPE_SCALE_LABELS       5
#define SCOPE_RANGE_HEADROOM     0.10f
#define SCOPE_RANGE_HYSTERESIS   0.05f
#define SCOPE_RANGE_DIVISIONS    8
#define SCOPE_MULT_MA            100    /* chart units per mA */
#define SCOPE_MULT_MW            10     /* chart units per mW */
#define SCOPE_CHART_VALUE_LIMIT  1000000000
#define SCOPE_CURSOR_LABEL_PAD   6
/* Fixed readout widths so the flex rows don't shift as text length changes. */
#define SCOPE_MODE_MATRIX_WIDTH  120
#define SCOPE_RUN_BUTTON_WIDTH   110
#define SCOPE_LIVE_LABEL_WIDTH   190
#define SCOPE_DIV_LABEL_WIDTH    110
#define SCOPE_AVG_LABEL_WIDTH    160
#define SCOPE_PEAK_LABEL_WIDTH   150

/*====================
 * State
 *====================*/
typedef struct {
    float sum;
    float max;
    uint32_t count;
} point_acc_t;

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *header;
    lv_obj_t *chart_area;
    lv_obj_t *footer;
    lv_obj_t *chart;
    lv_obj_t *scale;
    lv_obj_t *vbus_label;
    lv_obj_t *state_label;
    lv_obj_t *drop_label;
    lv_obj_t *mode_matrix;
    lv_obj_t *run_button;
    lv_obj_t *run_label;
    lv_obj_t *live_label;
    lv_obj_t *div_label;
    lv_obj_t *avg_label;
    lv_obj_t *peak_label;
    lv_obj_t *cursor_label;
    lv_chart_series_t *series;
    lv_chart_cursor_t *cursor;
    lv_timer_t *timer;

    int32_t points_ma[SCOPE_CHART_MAX_POINTS];
    int32_t points_mw[SCOPE_CHART_MAX_POINTS];
    uint32_t point_count;
    uint32_t samples_per_point;     /* k */
    point_acc_t acc_ma;
    point_acc_t acc_mw;

    uint32_t next_seq;
    bool hold;
    bool have_sample;
    float live_ma;
    float live_mw;
    float vbus_v;
    float peak_ma;
    float peak_mw;

    bool range_valid;
    float range_lo;
    float range_hi;
    char scale_text[SCOPE_SCALE_LABELS][12];
    const char *scale_src[SCOPE_SCALE_LABELS + 1];

    int32_t cursor_point;
    uint32_t settings_gen;
    settings_t cfg;
} scope_state_t;

static scope_state_t s;

static const char *mode_map[] = {"mA", "mW", ""};

/*====================
 * Helpers
 *====================*/
static int32_t mode_mult(scope_mode_t mode)
{
    return (mode == SCOPE_MODE_MA) ? SCOPE_MULT_MA : SCOPE_MULT_MW;
}

static int32_t *active_points(void)
{
    return (s.cfg.mode == SCOPE_MODE_MA) ? s.points_ma : s.points_mw;
}

static int32_t value_to_chart(float value, scope_mode_t mode)
{
    float scaled = value * (float)mode_mult(mode);
    if (scaled > (float)SCOPE_CHART_VALUE_LIMIT) {
        scaled = (float)SCOPE_CHART_VALUE_LIMIT;
    } else if (scaled < -(float)SCOPE_CHART_VALUE_LIMIT) {
        scaled = -(float)SCOPE_CHART_VALUE_LIMIT;
    }
    return (int32_t)lroundf(scaled);
}

static float chart_to_value(int32_t chart_value, scope_mode_t mode)
{
    return (float)chart_value / (float)mode_mult(mode);
}

/* "123.4 mW" / "1.234 W" / "56.7 mA" / "2.345 A" */
static void format_value(char *buf, size_t len, float value, scope_mode_t mode)
{
    const char *milli = (mode == SCOPE_MODE_MA) ? "mA" : "mW";
    const char *whole = (mode == SCOPE_MODE_MA) ? "A" : "W";

    if (fabsf(value) >= 1000.0f) {
        snprintf(buf, len, "%.3f %s", value / 1000.0f, whole);
    } else {
        snprintf(buf, len, "%.1f %s", value, milli);
    }
}

static void format_div(char *buf, size_t len, uint32_t window_s)
{
    float per_div = (float)window_s / (float)SCOPE_X_DIVISIONS;
    if (per_div < 1.0f) {
        snprintf(buf, len, "%.0f ms/div", per_div * 1000.0f);
    } else {
        snprintf(buf, len, "%.0f s/div", per_div);
    }
}

/* Round up to 1, 2 or 5 x 10^n. */
static float nice_step(float raw)
{
    float mag;
    float norm;

    if (raw <= 0.0f) {
        return 1.0f;
    }
    mag = powf(10.0f, floorf(log10f(raw)));
    norm = raw / mag;
    if (norm <= 1.0f) {
        return 1.0f * mag;
    }
    if (norm <= 2.0f) {
        return 2.0f * mag;
    }
    if (norm <= 5.0f) {
        return 5.0f * mag;
    }
    return 10.0f * mag;
}

/*====================
 * Y range and scale
 *====================*/
static void apply_range(float lo, float hi, bool force)
{
    int32_t lo_i;
    int32_t hi_i;
    float step;
    int decimals;

    if (!(hi > lo)) {
        hi = lo + 1.0f;
    }

    if (!force && s.range_valid) {
        float span = s.range_hi - s.range_lo;
        if (fabsf(lo - s.range_lo) <= span * SCOPE_RANGE_HYSTERESIS &&
            fabsf(hi - s.range_hi) <= span * SCOPE_RANGE_HYSTERESIS) {
            return;
        }
    }

    s.range_lo = lo;
    s.range_hi = hi;
    s.range_valid = true;

    lo_i = value_to_chart(lo, s.cfg.mode);
    hi_i = value_to_chart(hi, s.cfg.mode);
    if (hi_i <= lo_i) {
        hi_i = lo_i + 1;
    }
    lv_chart_set_axis_range(s.chart, LV_CHART_AXIS_PRIMARY_Y, lo_i, hi_i);
    lv_scale_set_range(s.scale, lo_i, hi_i);

    step = (hi - lo) / (float)(SCOPE_SCALE_LABELS - 1);
    if (step >= 10.0f) {
        decimals = 0;
    } else if (step >= 1.0f) {
        decimals = 1;
    } else {
        decimals = 2;
    }

    for (uint32_t i = 0U; i < SCOPE_SCALE_LABELS; i++) {
        float v = lo + step * (float)i;
        if (fabsf(v) >= 10000.0f) {
            snprintf(s.scale_text[i], sizeof(s.scale_text[i]), "%.1fk", v / 1000.0f);
        } else {
            snprintf(s.scale_text[i], sizeof(s.scale_text[i]), "%.*f", decimals, v);
        }
        s.scale_src[i] = s.scale_text[i];
    }
    s.scale_src[SCOPE_SCALE_LABELS] = NULL;
    lv_scale_set_text_src(s.scale, s.scale_src);
    lv_obj_invalidate(s.scale);
}

static void update_auto_range(void)
{
    const int32_t *points = active_points();
    int32_t min_i = INT32_MAX;
    int32_t max_i = INT32_MIN;
    float min_v;
    float max_v;
    float span;
    float step;
    float lo;
    float hi;

    for (uint32_t i = 0U; i < s.point_count; i++) {
        int32_t v = points[i];
        if (v == LV_CHART_POINT_NONE) {
            continue;
        }
        if (v < min_i) {
            min_i = v;
        }
        if (v > max_i) {
            max_i = v;
        }
    }
    if (min_i == INT32_MAX) {
        return;
    }

    min_v = chart_to_value(min_i, s.cfg.mode);
    max_v = chart_to_value(max_i, s.cfg.mode);
    span = max_v - min_v;
    if (span < 1.0f / (float)mode_mult(s.cfg.mode) * 4.0f) {
        span = 1.0f / (float)mode_mult(s.cfg.mode) * 4.0f;
    }
    lo = min_v - span * SCOPE_RANGE_HEADROOM;
    hi = max_v + span * SCOPE_RANGE_HEADROOM;
    /* Don't push a strictly positive signal below zero. */
    if (min_v >= 0.0f && lo < 0.0f) {
        lo = 0.0f;
    }

    step = nice_step((hi - lo) / (float)SCOPE_RANGE_DIVISIONS);
    lo = floorf(lo / step) * step;
    hi = ceilf(hi / step) * step;
    if (hi <= lo) {
        hi = lo + step;
    }
    apply_range(lo, hi, false);
}

static void refresh_range(bool force)
{
    if (s.cfg.y_auto) {
        if (force) {
            s.range_valid = false;
        }
        update_auto_range();
    } else {
        apply_range(s.cfg.y_min, s.cfg.y_max, force);
    }
}

/*====================
 * Data path
 *====================*/
static void reset_acc(void)
{
    memset(&s.acc_ma, 0, sizeof(s.acc_ma));
    memset(&s.acc_mw, 0, sizeof(s.acc_mw));
    s.acc_ma.max = -INFINITY;
    s.acc_mw.max = -INFINITY;
}

static void clear_points(void)
{
    for (uint32_t i = 0U; i < SCOPE_CHART_MAX_POINTS; i++) {
        s.points_ma[i] = LV_CHART_POINT_NONE;
        s.points_mw[i] = LV_CHART_POINT_NONE;
    }
    reset_acc();
}

static void reset_peak(void)
{
    s.peak_ma = -INFINITY;
    s.peak_mw = -INFINITY;
}

static void acc_add(point_acc_t *acc, float v)
{
    acc->sum += v;
    if (v > acc->max) {
        acc->max = v;
    }
    acc->count++;
}

static float acc_result(const point_acc_t *acc)
{
    if (s.cfg.decimation == SCOPE_DECIMATE_MAX) {
        return acc->max;
    }
    return acc->sum / (float)acc->count;
}

static void emit_point(void)
{
    uint32_t n = s.point_count;

    if (n == 0U) {
        return;
    }
    memmove(&s.points_ma[0], &s.points_ma[1], (n - 1U) * sizeof(int32_t));
    memmove(&s.points_mw[0], &s.points_mw[1], (n - 1U) * sizeof(int32_t));
    s.points_ma[n - 1U] = value_to_chart(acc_result(&s.acc_ma), SCOPE_MODE_MA);
    s.points_mw[n - 1U] = value_to_chart(acc_result(&s.acc_mw), SCOPE_MODE_MW);
    reset_acc();
}

static void process_sample(const sample_t *sample)
{
    s.live_ma = sample->current_ma;
    s.live_mw = sample->power_mw;
    s.vbus_v = sample->vbus_v;
    s.have_sample = true;
    if (sample->current_ma > s.peak_ma) {
        s.peak_ma = sample->current_ma;
    }
    if (sample->power_mw > s.peak_mw) {
        s.peak_mw = sample->power_mw;
    }

    acc_add(&s.acc_ma, sample->current_ma);
    acc_add(&s.acc_mw, sample->power_mw);
    if (s.acc_ma.count >= s.samples_per_point) {
        emit_point();
    }
}

static uint32_t window_samples(void)
{
    return s.point_count * s.samples_per_point;
}

/* Rebuild the whole trace from ring history so the display shows "now". */
static void rebuild_from_ring(void)
{
    sample_ring_t *ring = sensor_task_samples();
    uint32_t head = sample_ring_head(ring);
    uint32_t tail = sample_ring_tail(ring);
    uint32_t available = head - tail;
    uint32_t want = window_samples();
    uint32_t start = head - ((available < want) ? available : want);
    sample_t sample;

    clear_points();
    for (uint32_t seq = start; seq != head; seq++) {
        if (sample_ring_get(ring, seq, &sample)) {
            process_sample(&sample);
        }
    }
    s.next_seq = head;
}

/* Derive point count / decimation from settings and the chart width. */
static void configure_geometry(void)
{
    uint32_t chart_px = (uint32_t)lv_obj_get_content_width(s.chart);
    uint32_t max_points = (chart_px == 0U || chart_px > SCOPE_CHART_MAX_POINTS)
                          ? SCOPE_CHART_MAX_POINTS : chart_px;
    uint32_t ws = (uint32_t)s.cfg.window_s * s.cfg.sample_rate_hz;
    uint32_t k = (ws + max_points - 1U) / max_points;

    if (k == 0U) {
        k = 1U;
    }
    s.samples_per_point = k;
    s.point_count = ws / k;
    if (s.point_count == 0U) {
        s.point_count = 1U;
    }
    if (s.point_count > max_points) {
        s.point_count = max_points;
    }
    lv_chart_set_point_count(s.chart, s.point_count);
}

static void update_labels(void);

static void apply_settings(const settings_t *old)
{
    bool mode_changed = (old->mode != s.cfg.mode);
    bool geometry_changed = (old->window_s != s.cfg.window_s) ||
                            (old->sample_rate_hz != s.cfg.sample_rate_hz);
    bool decimation_changed = (old->decimation != s.cfg.decimation);
    bool range_changed = (old->y_auto != s.cfg.y_auto) ||
                         (old->y_min != s.cfg.y_min) ||
                         (old->y_max != s.cfg.y_max);
    char buf[24];

    lv_timer_set_period(s.timer, 1000U / s.cfg.refresh_hz);

    if (mode_changed) {
        lv_chart_set_series_ext_y_array(s.chart, s.series, active_points());
        lv_buttonmatrix_set_button_ctrl(s.mode_matrix, (uint32_t)s.cfg.mode,
                                        LV_BUTTONMATRIX_CTRL_CHECKED);
        reset_peak();
    }
    if (geometry_changed || decimation_changed) {
        configure_geometry();
        rebuild_from_ring();
        reset_peak();
        format_div(buf, sizeof(buf), s.cfg.window_s);
        lv_label_set_text(s.div_label, buf);
    }
    if (mode_changed || geometry_changed || range_changed || decimation_changed) {
        refresh_range(true);
        lv_chart_refresh(s.chart);
        update_labels();
    }
}

/*====================
 * Labels
 *====================*/
static void update_cursor_label(void)
{
    const int32_t *points = active_points();
    char value[24];
    float rel_s;
    lv_point_t p;
    int32_t x;
    int32_t width;
    int32_t chart_w;

    if (s.cursor_point == LV_CHART_POINT_NONE ||
        (uint32_t)s.cursor_point >= s.point_count) {
        lv_obj_add_flag(s.cursor_label, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (points[s.cursor_point] == LV_CHART_POINT_NONE) {
        snprintf(value, sizeof(value), "--");
    } else {
        format_value(value, sizeof(value),
                     chart_to_value(points[s.cursor_point], s.cfg.mode), s.cfg.mode);
    }
    rel_s = -((float)(s.point_count - 1U - (uint32_t)s.cursor_point) *
              (float)s.samples_per_point) / (float)s.cfg.sample_rate_hz;
    /* LVGL's builtin sprintf has no %f; format with the C library. */
    {
        char text[40];
        snprintf(text, sizeof(text), "%s\n%.2f s", value, (double)rel_s);
        lv_label_set_text(s.cursor_label, text);
    }

    lv_chart_get_point_pos_by_id(s.chart, s.series, (uint32_t)s.cursor_point, &p);
    lv_obj_update_layout(s.cursor_label);
    width = lv_obj_get_width(s.cursor_label);
    chart_w = lv_obj_get_width(s.chart);
    x = p.x - width / 2;
    if (x < SCOPE_CURSOR_LABEL_PAD) {
        x = SCOPE_CURSOR_LABEL_PAD;
    }
    if (x + width > chart_w - SCOPE_CURSOR_LABEL_PAD) {
        x = chart_w - SCOPE_CURSOR_LABEL_PAD - width;
    }
    lv_obj_set_pos(s.cursor_label, x, SCOPE_CURSOR_LABEL_PAD);
    lv_obj_remove_flag(s.cursor_label, LV_OBJ_FLAG_HIDDEN);
}

static void update_labels(void)
{
    const int32_t *points = active_points();
    char buf[24];
    float live = (s.cfg.mode == SCOPE_MODE_MA) ? s.live_ma : s.live_mw;
    float peak = (s.cfg.mode == SCOPE_MODE_MA) ? s.peak_ma : s.peak_mw;
    double sum = 0.0;
    uint32_t count = 0U;

    if (!s.have_sample) {
        lv_label_set_text(s.live_label, "--");
        lv_label_set_text(s.avg_label, "avg --");
        lv_label_set_text(s.peak_label, "pk --");
        return;
    }

    format_value(buf, sizeof(buf), live, s.cfg.mode);
    lv_label_set_text(s.live_label, buf);

    for (uint32_t i = 0U; i < s.point_count; i++) {
        if (points[i] != LV_CHART_POINT_NONE) {
            sum += (double)points[i];
            count++;
        }
    }
    if (count > 0U) {
        format_value(buf, sizeof(buf),
                     chart_to_value((int32_t)(sum / (double)count), s.cfg.mode),
                     s.cfg.mode);
        lv_label_set_text_fmt(s.avg_label, "avg %s", buf);
    } else {
        lv_label_set_text(s.avg_label, "avg --");
    }

    if (peak > -INFINITY) {
        format_value(buf, sizeof(buf), peak, s.cfg.mode);
        lv_label_set_text_fmt(s.peak_label, "pk %s", buf);
    } else {
        lv_label_set_text(s.peak_label, "pk --");
    }

    update_cursor_label();
}

static void update_status(void)
{
    uint32_t dropped = sample_ring_dropped(sensor_task_samples());

    if (s.have_sample) {
        char text[24];
        snprintf(text, sizeof(text), "VBUS %.2f V", (double)s.vbus_v);
        lv_label_set_text(s.vbus_label, text);
    } else {
        lv_label_set_text(s.vbus_label, "VBUS --.-- V");
    }

    if (dropped != 0U) {
        lv_label_set_text_fmt(s.drop_label, "drop %lu", (unsigned long)dropped);
        lv_obj_remove_flag(s.drop_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s.drop_label, LV_OBJ_FLAG_HIDDEN);
    }
}

static void set_hold(bool hold)
{
    s.hold = hold;
    if (hold) {
        lv_label_set_text(s.run_label, LV_SYMBOL_PLAY " RUN");
        lv_label_set_text(s.state_label, LV_SYMBOL_PAUSE " HOLD");
        lv_obj_set_style_text_color(s.state_label, lv_color_hex(UI_COLOR_HOLD), 0);
        lv_obj_add_state(s.run_button, LV_STATE_CHECKED);
    } else {
        lv_label_set_text(s.run_label, LV_SYMBOL_PAUSE " HOLD");
        lv_label_set_text(s.state_label, LV_SYMBOL_PLAY " RUN");
        lv_obj_set_style_text_color(s.state_label, lv_color_hex(UI_COLOR_RUN), 0);
        lv_obj_remove_state(s.run_button, LV_STATE_CHECKED);
        rebuild_from_ring();
        refresh_range(false);
        lv_chart_refresh(s.chart);
        update_labels();
    }
}

/*====================
 * Update timer (runs in the LVGL task)
 *====================*/
static void scope_tick(lv_timer_t *timer)
{
    sample_ring_t *ring = sensor_task_samples();
    uint32_t gen = settings_generation();
    uint32_t head;
    uint32_t retain;
    bool new_data = false;

    (void)timer;

    if (gen != s.settings_gen) {
        settings_t old = s.cfg;
        s.settings_gen = gen;
        settings_get(&s.cfg);
        apply_settings(&old);
    }

    head = sample_ring_head(ring);
    retain = window_samples();

    if (s.hold) {
        /* Keep the VBUS readout live, discard everything else. */
        sample_t latest;
        if (head != 0U && sample_ring_get(ring, head - 1U, &latest)) {
            s.vbus_v = latest.vbus_v;
            s.have_sample = true;
        }
        s.next_seq = head;
    } else {
        uint32_t lag = head - s.next_seq;
        sample_t sample;

        /* Bound the work: never process more than one window per tick. */
        if (lag > retain) {
            s.next_seq = head - retain;
            reset_acc();
        }
        for (uint32_t seq = s.next_seq; seq != head; seq++) {
            if (sample_ring_get(ring, seq, &sample)) {
                process_sample(&sample);
                new_data = true;
            }
        }
        s.next_seq = head;
    }

    /* Retain one window of history so RUN/mode/window changes can refill. */
    sample_ring_consume(ring, (head > retain) ? (head - retain) : 0U);

    if (new_data) {
        refresh_range(false);
        lv_chart_refresh(s.chart);
        update_labels();
    }
    update_status();
}

/*====================
 * Events
 *====================*/
static void mode_event_cb(lv_event_t *e)
{
    uint32_t id = lv_buttonmatrix_get_selected_button(lv_event_get_target_obj(e));
    settings_t cfg;

    if (id == LV_BUTTONMATRIX_BUTTON_NONE) {
        return;
    }
    settings_get(&cfg);
    cfg.mode = (id == 0U) ? SCOPE_MODE_MA : SCOPE_MODE_MW;
    settings_set(&cfg);
}

static void run_event_cb(lv_event_t *e)
{
    (void)e;
    set_hold(!s.hold);
}

static void settings_button_cb(lv_event_t *e)
{
    (void)e;
    ui_show_settings();
}

static void peak_event_cb(lv_event_t *e)
{
    (void)e;
    reset_peak();
    update_labels();
}

static void hide_cursor(void)
{
    if (s.cursor_point == LV_CHART_POINT_NONE) {
        return;
    }
    s.cursor_point = LV_CHART_POINT_NONE;
    lv_chart_set_cursor_point(s.chart, s.cursor, NULL, LV_CHART_POINT_NONE);
    lv_obj_add_flag(s.cursor_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(s.chart);
}

/* The chart itself only latches the pressed point on LV_EVENT_PRESSED, so
 * derive the index from the touch position to follow drags as well. */
static void chart_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev;
    lv_point_t p;
    lv_area_t coords;
    int32_t w;
    int32_t x;
    int32_t id;

    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) {
        return;
    }
    indev = lv_indev_active();
    if (indev == NULL || s.point_count < 2U) {
        return;
    }

    lv_indev_get_point(indev, &p);
    lv_obj_get_coords(s.chart, &coords);
    w = lv_obj_get_content_width(s.chart);
    if (w <= 0) {
        return;
    }
    x = p.x - coords.x1 - lv_obj_get_style_pad_left(s.chart, LV_PART_MAIN)
        - lv_obj_get_style_border_width(s.chart, LV_PART_MAIN);
    id = (x * (int32_t)(s.point_count - 1U) + w / 2) / w;
    if (id < 0) {
        id = 0;
    }
    if (id >= (int32_t)s.point_count) {
        id = (int32_t)s.point_count - 1;
    }

    if (id != s.cursor_point) {
        s.cursor_point = id;
        lv_chart_set_cursor_point(s.chart, s.cursor, s.series, (uint32_t)id);
        update_cursor_label();
    }
}

static void background_click_cb(lv_event_t *e)
{
    /* Only react to taps on the container itself, not on its children. */
    if (lv_event_get_target(e) == lv_event_get_current_target(e)) {
        hide_cursor();
    }
}

static void gesture_event_cb(lv_event_t *e)
{
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
    (void)e;
    if (dir == LV_DIR_LEFT) {
        ui_show_settings();
    }
}

/*====================
 * Construction
 *====================*/
static void build_header(void)
{
    lv_obj_t *button;
    lv_obj_t *title;
    lv_obj_t *right;

    s.header = ui_style_create_panel(s.screen);
    lv_obj_set_size(s.header, LV_PCT(100), SCOPE_HEADER_HEIGHT);
    lv_obj_set_pos(s.header, 0, 0);
    lv_obj_add_event_cb(s.header, background_click_cb, LV_EVENT_CLICKED, NULL);

    button = ui_style_create_button(s.header, LV_SYMBOL_SETTINGS " Settings", NULL);
    lv_obj_align(button, LV_ALIGN_LEFT_MID, SCOPE_MARGIN, 0);
    lv_obj_add_event_cb(button, settings_button_cb, LV_EVENT_CLICKED, NULL);

    title = ui_style_create_label(s.header, "POWER SCOPE", UI_FONT_TITLE, UI_COLOR_TEXT);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);
    if (LV_HOR_RES < 640) {
        lv_obj_add_flag(title, LV_OBJ_FLAG_HIDDEN);
    }

    right = lv_obj_create(s.header);
    lv_obj_remove_style_all(right);
    lv_obj_set_size(right, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, 12, 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, -SCOPE_MARGIN, 0);

    s.drop_label = ui_style_create_label(right, "", UI_FONT_SMALL, UI_COLOR_ERROR);
    lv_obj_add_flag(s.drop_label, LV_OBJ_FLAG_HIDDEN);
    s.vbus_label = ui_style_create_label(right, "VBUS --.--- V", UI_FONT_BODY, UI_COLOR_TEXT);
    s.state_label = ui_style_create_label(right, LV_SYMBOL_PLAY " RUN", UI_FONT_BODY, UI_COLOR_RUN);
}

static void build_chart(void)
{
    int32_t area_h = LV_VER_RES - SCOPE_HEADER_HEIGHT - SCOPE_FOOTER_HEIGHT;
    int32_t chart_w = LV_HOR_RES - SCOPE_SCALE_WIDTH - 2 * SCOPE_MARGIN;
    int32_t chart_h = area_h - 2 * SCOPE_MARGIN;

    s.chart_area = lv_obj_create(s.screen);
    lv_obj_remove_style_all(s.chart_area);
    lv_obj_set_size(s.chart_area, LV_PCT(100), area_h);
    lv_obj_set_pos(s.chart_area, 0, SCOPE_HEADER_HEIGHT);
    lv_obj_add_flag(s.chart_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s.chart_area, background_click_cb, LV_EVENT_CLICKED, NULL);

    s.scale = lv_scale_create(s.chart_area);
    lv_scale_set_mode(s.scale, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_obj_set_size(s.scale, SCOPE_SCALE_WIDTH, chart_h);
    lv_obj_set_pos(s.scale, 0, SCOPE_MARGIN);
    lv_scale_set_total_tick_count(s.scale, SCOPE_SCALE_TICKS);
    lv_scale_set_major_tick_every(s.scale, SCOPE_SCALE_MAJOR_EVERY);
    lv_scale_set_label_show(s.scale, true);
    lv_obj_set_style_pad_all(s.scale, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s.scale, 1, LV_PART_MAIN); /* match chart border */
    lv_obj_set_style_bg_opa(s.scale, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s.scale, 0, LV_PART_MAIN);
    lv_obj_set_style_line_color(s.scale, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_line_width(s.scale, 1, LV_PART_MAIN);
    lv_obj_set_style_line_color(s.scale, lv_color_hex(UI_COLOR_TEXT_DIM), LV_PART_INDICATOR);
    lv_obj_set_style_line_width(s.scale, 1, LV_PART_INDICATOR);
    lv_obj_set_style_length(s.scale, 8, LV_PART_INDICATOR);
    lv_obj_set_style_line_color(s.scale, lv_color_hex(UI_COLOR_BORDER), LV_PART_ITEMS);
    lv_obj_set_style_line_width(s.scale, 1, LV_PART_ITEMS);
    lv_obj_set_style_length(s.scale, 4, LV_PART_ITEMS);
    lv_obj_set_style_text_font(s.scale, UI_FONT_SMALL, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(s.scale, lv_color_hex(UI_COLOR_TEXT_DIM), LV_PART_INDICATOR);
    lv_obj_set_style_pad_hor(s.scale, 4, LV_PART_INDICATOR);

    s.chart = lv_chart_create(s.chart_area);
    lv_obj_set_size(s.chart, chart_w, chart_h);
    lv_obj_set_pos(s.chart, SCOPE_SCALE_WIDTH, SCOPE_MARGIN);
    lv_chart_set_type(s.chart, LV_CHART_TYPE_LINE);
    lv_chart_set_update_mode(s.chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_set_div_line_count(s.chart, SCOPE_CHART_HDIV, SCOPE_CHART_VDIV);
    lv_obj_set_style_pad_all(s.chart, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s.chart, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s.chart, lv_color_hex(UI_COLOR_CHART_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s.chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(s.chart, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_border_width(s.chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_color(s.chart, lv_color_hex(UI_COLOR_GRID), LV_PART_MAIN);
    lv_obj_set_style_line_width(s.chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_width(s.chart, SCOPE_CHART_LINE_WIDTH, LV_PART_ITEMS);
    lv_obj_set_style_size(s.chart, 0, 0, LV_PART_INDICATOR);   /* hide point markers */
    lv_obj_set_style_line_color(s.chart, lv_color_hex(UI_COLOR_CURSOR), LV_PART_CURSOR);
    lv_obj_set_style_line_width(s.chart, 1, LV_PART_CURSOR);
    lv_obj_remove_flag(s.chart, LV_OBJ_FLAG_SCROLLABLE);
    /* Drags on the chart move the cursor; swipes elsewhere switch screens. */
    lv_obj_remove_flag(s.chart, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(s.chart, chart_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(s.chart, chart_event_cb, LV_EVENT_PRESSING, NULL);

    s.series = lv_chart_add_series(s.chart, lv_color_hex(UI_COLOR_ACCENT),
                                   LV_CHART_AXIS_PRIMARY_Y);
    s.cursor = lv_chart_add_cursor(s.chart, lv_color_hex(UI_COLOR_CURSOR), LV_DIR_VER);
    s.cursor_point = LV_CHART_POINT_NONE;

    s.cursor_label = ui_style_create_label(s.chart, "", UI_FONT_SMALL, UI_COLOR_CURSOR);
    lv_obj_set_style_bg_color(s.cursor_label, lv_color_hex(UI_COLOR_PANEL), 0);
    lv_obj_set_style_bg_opa(s.cursor_label, LV_OPA_80, 0);
    lv_obj_set_style_pad_all(s.cursor_label, 4, 0);
    lv_obj_set_style_radius(s.cursor_label, 4, 0);
    lv_obj_set_style_text_align(s.cursor_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(s.cursor_label, LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_remove_flag(s.cursor_label, LV_OBJ_FLAG_CLICKABLE);
}

static void build_footer(void)
{
    lv_obj_t *row1;
    lv_obj_t *row2;
    bool two_rows = (LV_HOR_RES < 640);

    s.footer = ui_style_create_panel(s.screen);
    lv_obj_set_size(s.footer, LV_PCT(100), SCOPE_FOOTER_HEIGHT);
    lv_obj_set_pos(s.footer, 0, LV_VER_RES - SCOPE_FOOTER_HEIGHT);
    lv_obj_set_flex_flow(s.footer, two_rows ? LV_FLEX_FLOW_COLUMN : LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s.footer, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_hor(s.footer, SCOPE_MARGIN, 0);
    lv_obj_add_event_cb(s.footer, background_click_cb, LV_EVENT_CLICKED, NULL);

    if (two_rows) {
        row1 = lv_obj_create(s.footer);
        lv_obj_remove_style_all(row1);
        lv_obj_set_size(row1, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        row2 = lv_obj_create(s.footer);
        lv_obj_remove_style_all(row2);
        lv_obj_set_size(row2, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_add_event_cb(row1, background_click_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(row2, background_click_cb, LV_EVENT_CLICKED, NULL);
    } else {
        row1 = s.footer;
        row2 = s.footer;
    }

    s.mode_matrix = lv_buttonmatrix_create(row1);
    lv_buttonmatrix_set_map(s.mode_matrix, mode_map);
    lv_buttonmatrix_set_button_ctrl(s.mode_matrix, 0, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_button_ctrl(s.mode_matrix, 1, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(s.mode_matrix, true);
    lv_obj_set_size(s.mode_matrix, SCOPE_MODE_MATRIX_WIDTH, 40);
    lv_obj_set_style_pad_all(s.mode_matrix, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(s.mode_matrix, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s.mode_matrix, lv_color_hex(UI_COLOR_PANEL_ALT), LV_PART_MAIN);
    lv_obj_set_style_border_color(s.mode_matrix, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_border_width(s.mode_matrix, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s.mode_matrix, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s.mode_matrix, lv_color_hex(UI_COLOR_PANEL_ALT), LV_PART_ITEMS);
    lv_obj_set_style_text_color(s.mode_matrix, lv_color_hex(UI_COLOR_TEXT), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(s.mode_matrix, lv_color_hex(UI_COLOR_ACCENT),
                              LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(s.mode_matrix, lv_color_hex(UI_COLOR_BG),
                                LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_radius(s.mode_matrix, 4, LV_PART_ITEMS);
    lv_obj_add_event_cb(s.mode_matrix, mode_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    s.run_button = ui_style_create_button(row1, LV_SYMBOL_PAUSE " HOLD", &s.run_label);
    lv_obj_set_width(s.run_button, SCOPE_RUN_BUTTON_WIDTH);
    lv_obj_add_event_cb(s.run_button, run_event_cb, LV_EVENT_CLICKED, NULL);

    s.live_label = ui_style_create_label(row1, "--", UI_FONT_READOUT, UI_COLOR_ACCENT);
    lv_obj_set_width(s.live_label, SCOPE_LIVE_LABEL_WIDTH);
    lv_obj_set_style_text_align(s.live_label, LV_TEXT_ALIGN_RIGHT, 0);

    s.div_label = ui_style_create_label(row2, "1 s/div", UI_FONT_BODY, UI_COLOR_TEXT_DIM);
    lv_obj_set_width(s.div_label, SCOPE_DIV_LABEL_WIDTH);
    s.avg_label = ui_style_create_label(row2, "avg --", UI_FONT_BODY, UI_COLOR_TEXT_DIM);
    lv_obj_set_width(s.avg_label, SCOPE_AVG_LABEL_WIDTH);
    lv_obj_set_style_text_align(s.avg_label, LV_TEXT_ALIGN_CENTER, 0);
    s.peak_label = ui_style_create_label(row2, "pk --", UI_FONT_BODY, UI_COLOR_TEXT);
    lv_obj_set_width(s.peak_label, SCOPE_PEAK_LABEL_WIDTH);
    lv_obj_set_style_text_align(s.peak_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_add_flag(s.peak_label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s.peak_label, peak_event_cb, LV_EVENT_LONG_PRESSED, NULL);
}

lv_obj_t *ui_scope_create(void)
{
    settings_t initial;
    char buf[24];

    memset(&s, 0, sizeof(s));
    s.cursor_point = LV_CHART_POINT_NONE;
    reset_peak();

    s.screen = lv_obj_create(NULL);
    lv_obj_add_style(s.screen, &ui_style_screen, 0);
    lv_obj_remove_flag(s.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s.screen, gesture_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(s.screen, background_click_cb, LV_EVENT_CLICKED, NULL);

    build_header();
    build_chart();
    build_footer();

    /* Initial configuration straight from settings. */
    settings_get(&s.cfg);
    s.settings_gen = settings_generation();
    lv_obj_update_layout(s.chart);
    configure_geometry();
    clear_points();
    lv_chart_set_series_ext_y_array(s.chart, s.series, active_points());
    lv_buttonmatrix_set_button_ctrl(s.mode_matrix, (uint32_t)s.cfg.mode,
                                    LV_BUTTONMATRIX_CTRL_CHECKED);
    format_div(buf, sizeof(buf), s.cfg.window_s);
    lv_label_set_text(s.div_label, buf);
    refresh_range(true);
    if (!s.range_valid) {
        /* No data yet: show a sensible default range until samples arrive. */
        initial = s.cfg;
        apply_range(0.0f, (initial.mode == SCOPE_MODE_MA) ? 100.0f : 1000.0f, true);
        s.range_valid = false;
    }
    set_hold(false);

    s.timer = lv_timer_create(scope_tick, 1000U / s.cfg.refresh_hz, NULL);
    return s.screen;
}
