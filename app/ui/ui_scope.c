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
/* Fixed layout for the 480 x 800 portrait panel. */
#define SCOPE_HEADER_HEIGHT      60
#define SCOPE_FOOTER_HEIGHT      100    /* two rows */
#define SCOPE_SCALE_WIDTH        62
#define SCOPE_MARGIN             6
/* Room above the chart for the Y unit caption; the top tick label also
 * straddles the chart's top edge. */
#define SCOPE_UNIT_HEIGHT        28
#define SCOPE_SCALE_MAJOR_LEN    8
#define SCOPE_SCALE_LABEL_PAD    4
#define SCOPE_CHART_MAX_POINTS   480    /* at most one per pixel of the panel width */
/* The chart draws the trace mapped onto 0..SCOPE_CHART_SPAN across the Y range,
 * so the range can go down to microamps without LVGL's int32 maths (value span
 * x chart height) overflowing. */
#define SCOPE_CHART_SPAN         10000
#define SCOPE_CHART_LINE_WIDTH   2
#define SCOPE_CHART_HDIV         9      /* 8 horizontal bands */
#define SCOPE_CHART_VDIV         11     /* 10 vertical bands (s/div) */
#define SCOPE_X_DIVISIONS        10
#define SCOPE_SCALE_TICKS        9
#define SCOPE_SCALE_MAJOR_EVERY  2
#define SCOPE_SCALE_LABELS       5
#define SCOPE_LABEL_INTERVALS    (SCOPE_SCALE_LABELS - 1)  /* 2 grid bands each */
#define SCOPE_RANGE_HEADROOM     0.10f
/* Smallest auto range, in display units: about four current LSBs (244 nA). */
#define SCOPE_MIN_SPAN_MA        0.001f     /* 1 uA */
#define SCOPE_MIN_SPAN_MW        0.01f      /* 10 uW */
/* Auto range grows at once but only shrinks/shifts after the smaller range has
 * been possible for max(window, this) so a spike doesn't rescale twice. */
#define SCOPE_RANGE_SHRINK_MIN_MS 3000U
/* Scope timer rate. At the default 10 s window a new point lands every ~25 ms,
 * so 30 Hz keeps up; faster only costs render time. */
#define SCOPE_REFRESH_HZ         30U
/* Warn when the achieved sample rate is below this % of configured. Ignore the
 * measurement for SCOPE_RATE_SETTLE_MS after a rate change (the sensor's 1 s
 * count straddles the change). */
#define SCOPE_RATE_LOW_PCT       90U
#define SCOPE_RATE_SETTLE_MS     2500U
/* A time bin with no sample repeats the previous point, unless no sample has
 * arrived for this long: then the sensor has stalled and the trace shows a gap. */
#define SCOPE_GAP_MS             1000U
/* History kept in the ring beyond the longest window, so a rebuild covers the
 * whole window despite timing jitter. */
#define SCOPE_HISTORY_SLACK_MS   1000U
#define SCOPE_CURSOR_LABEL_PAD   6
/* Fixed widths so the flex rows don't shift as text length changes. */
#define SCOPE_MODE_MATRIX_WIDTH  145
#define SCOPE_RUN_BUTTON_WIDTH   133
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

/* A display unit: how many mA (or mW) one of it is, and its name. The fonts
 * have no micro sign, hence "u". */
typedef struct {
    float per_unit;
    const char *name;
} scope_unit_t;

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *header;
    lv_obj_t *chart_area;
    lv_obj_t *footer;
    lv_obj_t *chart;
    lv_obj_t *scale;
    lv_obj_t *unit_label;
    lv_obj_t *vbus_label;
    lv_obj_t *drop_label;
    lv_obj_t *rate_label;
    lv_obj_t *state_label;
    lv_obj_t *run_button;
    lv_obj_t *run_label;
    lv_obj_t *mode_matrix;
    lv_obj_t *div_label;
    lv_obj_t *avg_label;
    lv_obj_t *peak_label;
    lv_obj_t *cursor_label;
    lv_chart_series_t *series;
    lv_chart_cursor_t *cursor;
    lv_timer_t *timer;

    /* One entry per time bin, oldest first. Values are in display units (mA,
     * mW) as the decimation picks them; NAN marks a bin with nothing to draw. */
    float points_ma[SCOPE_CHART_MAX_POINTS];
    float points_mw[SCOPE_CHART_MAX_POINTS];
    /* Raw sample sums and counts per bin, so the average readout is the mean
     * of the samples themselves whatever the decimation draws. */
    float sums_ma[SCOPE_CHART_MAX_POINTS];
    float sums_mw[SCOPE_CHART_MAX_POINTS];
    uint32_t counts[SCOPE_CHART_MAX_POINTS];
    /* The active trace mapped onto the Y range; this is what the chart draws. */
    int32_t chart_y[SCOPE_CHART_MAX_POINTS];
    uint32_t point_count;
    point_acc_t acc_ma;             /* samples in the bin being filled */
    point_acc_t acc_mw;
    /* The window is point_count equal time bins. Bin `bin_index` of the current
     * cycle ends at bin_origin_ms + (bin_index + 1) * window_ms / point_count;
     * one cycle spans exactly one window, so the origin steps by window_ms. */
    uint32_t bin_origin_ms;
    uint32_t bin_index;
    uint32_t last_sample_ms;        /* for the SCOPE_GAP_MS rule */
    bool have_last_sample;
    bool trace_changed;             /* a bin closed since the last redraw */
    uint16_t shown_window_s;        /* window the trace is built for; lags
                                     * cfg.window_s while a change waits for RUN */
    bool geometry_pending;          /* window/rate/decimation changed during HOLD */
    uint32_t rate_tick;             /* last rate change, for SCOPE_RATE_SETTLE_MS */

    uint32_t next_seq;
    bool hold;
    bool have_sample;
    float vbus_v;
    float peak_ma;
    float peak_mw;

    bool range_valid;
    float range_lo;
    float range_hi;
    bool shrink_pending;
    uint32_t shrink_tick;
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
static float *active_points(void)
{
    return (s.cfg.mode == SCOPE_MODE_MA) ? s.points_ma : s.points_mw;
}

/* Micro below 1 milli-unit, the base unit from 1000 up. */
static scope_unit_t pick_unit(float magnitude, scope_mode_t mode)
{
    bool ma = (mode == SCOPE_MODE_MA);
    scope_unit_t unit;

    if (magnitude < 1.0f) {
        unit.per_unit = 0.001f;
        unit.name = ma ? "uA" : "uW";
    } else if (magnitude < 1000.0f) {
        unit.per_unit = 1.0f;
        unit.name = ma ? "mA" : "mW";
    } else {
        unit.per_unit = 1000.0f;
        unit.name = ma ? "A" : "W";
    }
    return unit;
}

/* "245.3 uA" / "12.34 mA" / "1.234 A" */
static void format_value(char *buf, size_t len, float value, scope_mode_t mode)
{
    scope_unit_t unit = pick_unit(fabsf(value), mode);
    int decimals = (unit.per_unit < 1.0f) ? 1 : ((unit.per_unit > 1.0f) ? 3 : 2);

    snprintf(buf, len, "%.*f %s", decimals, (double)(value / unit.per_unit), unit.name);
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

/* Fewest decimals (up to 3) that print v exactly, allowing for float noise. */
static int decimals_for(float v)
{
    float a = fabsf(v);

    for (int d = 0; d < 3; d++) {
        float scaled = a * powf(10.0f, (float)d);
        float tolerance = 1.0e-3f * ((scaled > 1.0f) ? scaled : 1.0f);

        if (fabsf(scaled - roundf(scaled)) <= tolerance) {
            return d;
        }
    }
    return 3;
}

/*====================
 * Y range and scale
 *====================*/
static int32_t to_chart(float v)
{
    float span = s.range_hi - s.range_lo;
    float y;

    if (isnan(v) || !(span > 0.0f)) {
        return LV_CHART_POINT_NONE;
    }
    y = (v - s.range_lo) * (float)SCOPE_CHART_SPAN / span;
    /* lv_map() clamps to the axis anyway; this only keeps the conversion to
     * int32 defined for wild outliers. */
    if (y > 1.0e6f) {
        y = 1.0e6f;
    } else if (y < -1.0e6f) {
        y = -1.0e6f;
    }
    return (int32_t)lroundf(y);
}

static void remap_chart(void)
{
    const float *points = active_points();

    for (uint32_t i = 0U; i < s.point_count; i++) {
        s.chart_y[i] = to_chart(points[i]);
    }
}

static float min_span(void)
{
    return (s.cfg.mode == SCOPE_MODE_MA) ? SCOPE_MIN_SPAN_MA : SCOPE_MIN_SPAN_MW;
}

/* Set the Y range (display units), label the scale in a unit that suits its
 * magnitude, and redraw the trace against it. The caller refreshes the chart. */
static void apply_range(float lo, float hi)
{
    scope_unit_t unit;
    float step;
    int decimals;

    if (!(hi > lo)) {
        hi = lo + min_span();
    }
    s.range_lo = lo;
    s.range_hi = hi;
    s.range_valid = true;
    remap_chart();

    unit = pick_unit(fmaxf(fabsf(lo), fabsf(hi)), s.cfg.mode);
    step = (hi - lo) / (float)SCOPE_LABEL_INTERVALS;
    decimals = decimals_for(step / unit.per_unit);
    if (decimals_for(lo / unit.per_unit) > decimals) {
        decimals = decimals_for(lo / unit.per_unit);
    }
    for (uint32_t i = 0U; i < SCOPE_SCALE_LABELS; i++) {
        float v = (lo + step * (float)i) / unit.per_unit;

        if (fabsf(v) < 1.0e-6f) {
            v = 0.0f;       /* no "-0" */
        }
        snprintf(s.scale_text[i], sizeof(s.scale_text[i]), "%.*f", decimals, (double)v);
        s.scale_src[i] = s.scale_text[i];
    }
    s.scale_src[SCOPE_SCALE_LABELS] = NULL;
    lv_scale_set_text_src(s.scale, s.scale_src);
    lv_obj_invalidate(s.scale);
    lv_label_set_text(s.unit_label, unit.name);
}

/* Smallest range of SCOPE_LABEL_INTERVALS nice steps that covers the data plus
 * headroom, with lo on a step boundary so every label is a round number. */
static void ideal_auto_range(float min_v, float max_v, float *lo_out, float *hi_out,
                             float *step_out)
{
    float span = max_v - min_v;
    float lo;
    float hi;
    float step;
    float base;

    if (span < min_span()) {
        span = min_span();
    }
    lo = min_v - span * SCOPE_RANGE_HEADROOM;
    hi = max_v + span * SCOPE_RANGE_HEADROOM;
    /* Don't push a strictly positive signal below zero. */
    if (min_v >= 0.0f && lo < 0.0f) {
        lo = 0.0f;
    }

    step = nice_step((hi - lo) / (float)SCOPE_LABEL_INTERVALS);
    base = floorf(lo / step) * step;
    /* Flooring lo can cost up to one step; the next nice step always fits.
     * Bounded in case of float surprises. */
    for (uint32_t i = 0U; i < 4U && base + step * (float)SCOPE_LABEL_INTERVALS < hi; i++) {
        step = nice_step(step * 1.5f);
        base = floorf(lo / step) * step;
    }

    *lo_out = base;
    *hi_out = base + step * (float)SCOPE_LABEL_INTERVALS;
    *step_out = step;
}

/* Returns false when there is no data to range on. */
static bool update_auto_range(void)
{
    const float *points = active_points();
    float min_v = INFINITY;
    float max_v = -INFINITY;
    float step;
    float lo;
    float hi;
    uint32_t hold_ms;

    for (uint32_t i = 0U; i < s.point_count; i++) {
        float v = points[i];
        if (isnan(v)) {
            continue;
        }
        if (v < min_v) {
            min_v = v;
        }
        if (v > max_v) {
            max_v = v;
        }
    }
    if (min_v > max_v) {
        return false;
    }

    ideal_auto_range(min_v, max_v, &lo, &hi, &step);

    /* Grow at once: data outside the axis is worse than a rescale. */
    if (!s.range_valid || min_v < s.range_lo || max_v > s.range_hi) {
        s.shrink_pending = false;
        apply_range(lo, hi);
        return true;
    }
    if (fabsf(lo - s.range_lo) <= step * 0.01f && fabsf(hi - s.range_hi) <= step * 0.01f) {
        s.shrink_pending = false;
        return true;
    }

    /* Shrink/shift lazily, once the data has fit the smaller range for a while. */
    if (!s.shrink_pending) {
        s.shrink_pending = true;
        s.shrink_tick = lv_tick_get();
        return true;
    }
    hold_ms = (uint32_t)s.cfg.window_s * 1000U;
    if (hold_ms < SCOPE_RANGE_SHRINK_MIN_MS) {
        hold_ms = SCOPE_RANGE_SHRINK_MIN_MS;
    }
    if (lv_tick_elaps(s.shrink_tick) >= hold_ms) {
        s.shrink_pending = false;
        apply_range(lo, hi);
    }
    return true;
}

/* Until data arrives: a range in the right unit, replaced by the first data. */
static void apply_default_range(void)
{
    apply_range(0.0f, (s.cfg.mode == SCOPE_MODE_MA) ? 100.0f : 1000.0f);
    s.range_valid = false;
}

static void refresh_range(bool force)
{
    if (s.cfg.y_auto) {
        if (force) {
            s.range_valid = false;
            s.shrink_pending = false;
        }
        if (!update_auto_range() && force) {
            apply_default_range();
        }
    } else if (force || !s.range_valid) {
        /* Manual limits only change through settings, which force a refresh. */
        apply_range(s.cfg.y_min, s.cfg.y_max);
    }
}

/*====================
 * Data path
 *====================*/
#define SHIFT_LEFT(arr, n) memmove(&(arr)[0], &(arr)[1], ((n) - 1U) * sizeof((arr)[0]))

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
        s.points_ma[i] = NAN;
        s.points_mw[i] = NAN;
        s.sums_ma[i] = 0.0f;
        s.sums_mw[i] = 0.0f;
        s.counts[i] = 0U;
        s.chart_y[i] = LV_CHART_POINT_NONE;
    }
    reset_acc();
    s.have_last_sample = false;
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

/* Span of the trace on screen: bins and cursor times follow what is displayed,
 * which during HOLD may not be the latest setting yet. */
static uint32_t window_ms(void)
{
    return (uint32_t)s.shown_window_s * 1000U;
}

/* Ring history to keep: the longest selectable window plus slack. */
static uint32_t history_ms(void)
{
    return (uint32_t)settings_window_options_s[SETTINGS_WINDOW_COUNT - 1U] * 1000U +
           SCOPE_HISTORY_SLACK_MS;
}

/* Samples one window holds at the configured rate. */
static uint32_t window_samples(void)
{
    return (uint32_t)s.cfg.window_s * s.cfg.sample_rate_hz;
}

/* (bin_index + 1) * window_ms stays below 480 * 60000, well inside 32 bits. */
static uint32_t bin_end_ms(void)
{
    return s.bin_origin_ms + ((s.bin_index + 1U) * window_ms()) / s.point_count;
}

static void start_timeline(uint32_t start_ms)
{
    s.bin_origin_ms = start_ms;
    s.bin_index = 0U;
}

/* Append the bin being filled to the trace. An empty bin repeats the previous
 * point over a short dropout and becomes a gap once the sensor has been
 * silent for SCOPE_GAP_MS; either way it adds nothing to the average. */
static void close_bin(uint32_t end_ms)
{
    uint32_t n = s.point_count;
    uint32_t last = n - 1U;
    float ma = NAN;
    float mw = NAN;

    if (s.acc_ma.count > 0U) {
        ma = acc_result(&s.acc_ma);
        mw = acc_result(&s.acc_mw);
    } else if (s.have_last_sample && (end_ms - s.last_sample_ms) <= SCOPE_GAP_MS) {
        ma = s.points_ma[last];
        mw = s.points_mw[last];
    }
    SHIFT_LEFT(s.points_ma, n);
    SHIFT_LEFT(s.points_mw, n);
    SHIFT_LEFT(s.sums_ma, n);
    SHIFT_LEFT(s.sums_mw, n);
    SHIFT_LEFT(s.counts, n);
    SHIFT_LEFT(s.chart_y, n);
    s.points_ma[last] = ma;
    s.points_mw[last] = mw;
    s.sums_ma[last] = s.acc_ma.sum;
    s.sums_mw[last] = s.acc_mw.sum;
    s.counts[last] = s.acc_ma.count;
    s.chart_y[last] = to_chart((s.cfg.mode == SCOPE_MODE_MA) ? ma : mw);
    reset_acc();
    s.trace_changed = true;
}

/* Close every bin that ends at or before t_ms. All comparisons are signed
 * differences so they survive the ms clock wrapping. */
static void advance_to(uint32_t t_ms)
{
    uint32_t end = bin_end_ms();

    /* More than a window behind (the LVGL task stalled with no samples to
     * rebuild from): nothing on screen is current, so start over at t_ms. */
    if ((int32_t)(t_ms - end) >= (int32_t)window_ms()) {
        clear_points();
        start_timeline(t_ms);
        s.trace_changed = true;
        return;
    }
    while ((int32_t)(t_ms - end) >= 0) {
        close_bin(end);
        s.bin_index++;
        if (s.bin_index == s.point_count) {
            s.bin_index = 0U;
            s.bin_origin_ms += window_ms();
        }
        end = bin_end_ms();
    }
}

static void process_sample(const sample_t *sample)
{
    /* Place by timestamp, not by count, so rate changes and a sensor that
     * can't keep up still land on the right part of the time axis. */
    advance_to(sample->t_ms);

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
    s.last_sample_ms = sample->t_ms;
    s.have_last_sample = true;
}

/* Rebuild the whole trace from ring history so the display shows the window
 * ending now. Read the clock before head: a sample stamped after `now` is also
 * pushed after it (the sensor task stamps and pushes without blocking), so
 * closing bins up to `now` never closes one a later sample belongs in. */
static void rebuild_from_ring(void)
{
    sample_ring_t *ring = sensor_task_samples();
    uint32_t now = sensor_task_now_ms();
    uint32_t head = sample_ring_head(ring);
    uint32_t start_ms = now - window_ms();
    sample_t sample;

    clear_points();
    start_timeline(start_ms);
    for (uint32_t seq = sample_ring_seek(ring, head, start_ms); seq != head; seq++) {
        if (sample_ring_get(ring, seq, &sample)) {
            process_sample(&sample);
        }
    }
    advance_to(now);
    s.next_seq = head;
}

/* One point per chart pixel whatever the rate and window. With at least as
 * many points as pixels LVGL draws the trace as vertical runs (plain fills);
 * with fewer it switches to anti-aliased skewed segments, which made 20 Hz and
 * short windows render several times slower. Bins that get no sample repeat
 * the previous point (close_bin), so sparse data draws as sample-and-hold steps. */
static void configure_geometry(void)
{
    uint32_t chart_px = (uint32_t)lv_obj_get_content_width(s.chart);
    uint32_t n = (chart_px == 0U || chart_px > SCOPE_CHART_MAX_POINTS)
                 ? SCOPE_CHART_MAX_POINTS : chart_px;

    if (n < 2U) {
        n = 2U;     /* a line needs two points; also keeps bin_end_ms() defined */
    }
    s.point_count = n;
    s.shown_window_s = s.cfg.window_s;
    lv_chart_set_point_count(s.chart, n);
}

/* Size the trace for the current settings, with the matching s/div label. The
 * caller rebuilds the trace afterwards. */
static void apply_geometry(void)
{
    char buf[24];

    configure_geometry();
    reset_peak();
    format_div(buf, sizeof(buf), s.shown_window_s);
    lv_label_set_text(s.div_label, buf);
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

    if (old->sample_rate_hz != s.cfg.sample_rate_hz) {
        s.rate_tick = lv_tick_get();    /* re-measure before judging the rate */
    }

    if (mode_changed) {
        /* The forced range refresh below redraws the trace in the new unit. */
        lv_buttonmatrix_set_button_ctrl(s.mode_matrix, (uint32_t)s.cfg.mode,
                                        LV_BUTTONMATRIX_CTRL_CHECKED);
        reset_peak();
    }
    if (geometry_changed || decimation_changed) {
        if (s.hold) {
            /* Rebuilding now would replace the held capture with live data;
             * keep it (and its s/div) until RUN. */
            s.geometry_pending = true;
        } else {
            apply_geometry();
            rebuild_from_ring();
        }
    }
    if (mode_changed || geometry_changed || range_changed || decimation_changed) {
        refresh_range(true);
        lv_chart_refresh(s.chart);
        update_labels();
    }
}

/* achieved_hz is 0 until the sensor's first full second, and straddles the
 * old and new rate for a while after a change. */
static bool rate_measured(const sensor_stats_t *st)
{
    return (st->achieved_hz != 0U) && (lv_tick_elaps(s.rate_tick) >= SCOPE_RATE_SETTLE_MS);
}

static bool rate_is_low(const sensor_stats_t *st)
{
    return rate_measured(st) &&
           (st->achieved_hz * 100U < (uint32_t)s.cfg.sample_rate_hz * SCOPE_RATE_LOW_PCT);
}

/*====================
 * Labels
 *====================*/
static void update_cursor_label(void)
{
    const float *points = active_points();
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

    if (isnan(points[s.cursor_point])) {
        snprintf(value, sizeof(value), "--");
    } else {
        format_value(value, sizeof(value), points[s.cursor_point], s.cfg.mode);
    }
    /* Points are equal time bins; the newest one ends at the right edge. */
    rel_s = -(float)(s.point_count - 1U - (uint32_t)s.cursor_point) *
            (float)window_ms() / (float)s.point_count / 1000.0f;
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
    const float *sums = (s.cfg.mode == SCOPE_MODE_MA) ? s.sums_ma : s.sums_mw;
    char buf[24];
    float peak = (s.cfg.mode == SCOPE_MODE_MA) ? s.peak_ma : s.peak_mw;
    double sum = 0.0;
    uint32_t count = 0U;

    if (!s.have_sample) {
        lv_label_set_text(s.avg_label, "avg --");
        lv_label_set_text(s.peak_label, "pk --");
        return;
    }

    /* Mean of every sample in the window, not of the drawn points: with Max
     * decimation those are per-bin peaks. */
    for (uint32_t i = 0U; i < s.point_count; i++) {
        sum += (double)sums[i];
        count += s.counts[i];
    }
    if (count > 0U) {
        format_value(buf, sizeof(buf), (float)(sum / (double)count), s.cfg.mode);
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

static void update_status(const sensor_stats_t *st)
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

    if (rate_is_low(st)) {
        lv_label_set_text_fmt(s.rate_label, "%lu/%u Hz", (unsigned long)st->achieved_hz,
                              (unsigned int)s.cfg.sample_rate_hz);
        lv_obj_remove_flag(s.rate_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s.rate_label, LV_OBJ_FLAG_HIDDEN);
    }
}

/* The header label shows the current state; the footer button shows the
 * action it will take. */
static void set_hold(bool hold)
{
    s.hold = hold;
    if (hold) {
        lv_label_set_text(s.run_label, LV_SYMBOL_PLAY " RUN");
        lv_label_set_text(s.state_label, LV_SYMBOL_PAUSE " HOLD");
        lv_obj_set_style_text_color(s.state_label, lv_color_hex(UI_COLOR_HOLD), 0);
        lv_obj_add_state(s.run_button, LV_STATE_CHECKED);
    } else {
        bool deferred = s.geometry_pending;

        lv_label_set_text(s.run_label, LV_SYMBOL_PAUSE " HOLD");
        lv_label_set_text(s.state_label, LV_SYMBOL_PLAY " RUN");
        lv_obj_set_style_text_color(s.state_label, lv_color_hex(UI_COLOR_RUN), 0);
        lv_obj_remove_state(s.run_button, LV_STATE_CHECKED);
        if (deferred) {
            s.geometry_pending = false;
            apply_geometry();
        }
        rebuild_from_ring();
        refresh_range(deferred);    /* a settings change forces a fresh range */
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
    sensor_stats_t st;
    uint32_t now;
    uint32_t head;
    uint32_t keep_from;
    bool new_data = false;

    (void)timer;

    if (gen != s.settings_gen) {
        settings_t old = s.cfg;
        s.settings_gen = gen;
        settings_get(&s.cfg);
        apply_settings(&old);
    }

    sensor_task_get_stats(&st);

    /* Clock before head, as in rebuild_from_ring(). */
    now = sensor_task_now_ms();
    head = sample_ring_head(ring);

    if (s.hold) {
        /* Keep the VBUS readout live, discard everything else. */
        sample_t latest;
        if (head != 0U && sample_ring_get(ring, head - 1U, &latest)) {
            s.vbus_v = latest.vbus_v;
            s.have_sample = true;
        }
        s.next_seq = head;
    } else if (head - s.next_seq > window_samples()) {
        /* Fell more than a window behind: rebuilding is cheaper and exact. */
        rebuild_from_ring();
        new_data = true;
    } else {
        sample_t sample;

        for (uint32_t seq = s.next_seq; seq != head; seq++) {
            if (sample_ring_get(ring, seq, &sample)) {
                process_sample(&sample);
                new_data = true;
            }
        }
        s.next_seq = head;
        /* Scroll with real time even when no sample arrived, so a stalled
         * sensor shows up as a gap rather than a frozen trace. */
        advance_to(now);
    }

    /* Keep the longest window's worth of history whatever is on screen, by
     * time rather than count, so zooming out or changing the rate redraws from
     * it. 61 s at 333 Hz fits; the cap only guards the producer's headroom. */
    keep_from = sample_ring_seek(ring, head, now - history_ms());
    if (head - keep_from > SAMPLE_RING_CAPACITY - SAMPLE_RING_HEADROOM) {
        keep_from = head - (SAMPLE_RING_CAPACITY - SAMPLE_RING_HEADROOM);
    }
    sample_ring_consume(ring, keep_from);

    if (new_data || s.trace_changed) {
        s.trace_changed = false;
        refresh_range(false);
        lv_chart_refresh(s.chart);
        update_labels();
    }
    update_status(&st);
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
    lv_obj_t *right;

    s.header = ui_style_create_panel(s.screen);
    lv_obj_set_size(s.header, LV_PCT(100), SCOPE_HEADER_HEIGHT);
    lv_obj_set_pos(s.header, 0, 0);
    lv_obj_add_event_cb(s.header, background_click_cb, LV_EVENT_CLICKED, NULL);

    button = ui_style_create_button(s.header, LV_SYMBOL_SETTINGS " Settings", NULL);
    lv_obj_align(button, LV_ALIGN_LEFT_MID, SCOPE_MARGIN, 0);
    lv_obj_add_event_cb(button, settings_button_cb, LV_EVENT_CLICKED, NULL);

    right = lv_obj_create(s.header);
    lv_obj_remove_style_all(right);
    lv_obj_set_size(right, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(right, 12, 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, -SCOPE_MARGIN, 0);

    s.drop_label = ui_style_create_label(right, "", UI_FONT_SMALL, UI_COLOR_ERROR);
    lv_obj_add_flag(s.drop_label, LV_OBJ_FLAG_HIDDEN);
    s.rate_label = ui_style_create_label(right, "", UI_FONT_SMALL, UI_COLOR_HOLD);
    lv_obj_add_flag(s.rate_label, LV_OBJ_FLAG_HIDDEN);
    s.vbus_label = ui_style_create_label(right, "VBUS --.-- V", UI_FONT_BODY, UI_COLOR_TEXT);
    /* set_hold() fills in the text and colour. */
    s.state_label = ui_style_create_label(right, "", UI_FONT_BODY, UI_COLOR_RUN);
}

static void build_chart(void)
{
    int32_t area_h = LV_VER_RES - SCOPE_HEADER_HEIGHT - SCOPE_FOOTER_HEIGHT;
    int32_t chart_w = LV_HOR_RES - SCOPE_SCALE_WIDTH - 2 * SCOPE_MARGIN;
    int32_t chart_h = area_h - SCOPE_UNIT_HEIGHT - SCOPE_MARGIN;

    s.chart_area = lv_obj_create(s.screen);
    lv_obj_remove_style_all(s.chart_area);
    lv_obj_set_size(s.chart_area, LV_PCT(100), area_h);
    lv_obj_set_pos(s.chart_area, 0, SCOPE_HEADER_HEIGHT);
    lv_obj_add_flag(s.chart_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s.chart_area, background_click_cb, LV_EVENT_CLICKED, NULL);

    /* Y unit caption, right-aligned over the scale labels. */
    s.unit_label = ui_style_create_label(s.chart_area, "", UI_FONT_SMALL, UI_COLOR_TEXT_DIM);
    lv_obj_set_width(s.unit_label,
                     SCOPE_SCALE_WIDTH - SCOPE_SCALE_MAJOR_LEN - SCOPE_SCALE_LABEL_PAD);
    lv_obj_set_style_text_align(s.unit_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(s.unit_label, 0, SCOPE_MARGIN / 2);

    /* The scale and the chart both span 0..SCOPE_CHART_SPAN; apply_range()
     * supplies the labels for whatever that represents. */
    s.scale = lv_scale_create(s.chart_area);
    lv_scale_set_mode(s.scale, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_obj_set_size(s.scale, SCOPE_SCALE_WIDTH, chart_h);
    lv_obj_set_pos(s.scale, 0, SCOPE_UNIT_HEIGHT);
    lv_scale_set_range(s.scale, 0, SCOPE_CHART_SPAN);
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
    lv_obj_set_style_length(s.scale, SCOPE_SCALE_MAJOR_LEN, LV_PART_INDICATOR);
    lv_obj_set_style_line_color(s.scale, lv_color_hex(UI_COLOR_BORDER), LV_PART_ITEMS);
    lv_obj_set_style_line_width(s.scale, 1, LV_PART_ITEMS);
    lv_obj_set_style_length(s.scale, 4, LV_PART_ITEMS);
    lv_obj_set_style_text_font(s.scale, UI_FONT_SMALL, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(s.scale, lv_color_hex(UI_COLOR_TEXT_DIM), LV_PART_INDICATOR);
    lv_obj_set_style_pad_hor(s.scale, SCOPE_SCALE_LABEL_PAD, LV_PART_INDICATOR);

    s.chart = lv_chart_create(s.chart_area);
    lv_obj_set_size(s.chart, chart_w, chart_h);
    lv_obj_set_pos(s.chart, SCOPE_SCALE_WIDTH, SCOPE_UNIT_HEIGHT);
    lv_chart_set_type(s.chart, LV_CHART_TYPE_LINE);
    lv_chart_set_axis_range(s.chart, LV_CHART_AXIS_PRIMARY_Y, 0, SCOPE_CHART_SPAN);
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
    lv_chart_set_series_ext_y_array(s.chart, s.series, s.chart_y);
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

static lv_obj_t *create_footer_row(void)
{
    lv_obj_t *row = lv_obj_create(s.footer);

    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_add_event_cb(row, background_click_cb, LV_EVENT_CLICKED, NULL);
    return row;
}

static void build_footer(void)
{
    lv_obj_t *row1;
    lv_obj_t *row2;

    s.footer = ui_style_create_panel(s.screen);
    lv_obj_set_size(s.footer, LV_PCT(100), SCOPE_FOOTER_HEIGHT);
    lv_obj_set_pos(s.footer, 0, LV_VER_RES - SCOPE_FOOTER_HEIGHT);
    lv_obj_set_flex_flow(s.footer, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s.footer, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_hor(s.footer, SCOPE_MARGIN, 0);
    lv_obj_add_event_cb(s.footer, background_click_cb, LV_EVENT_CLICKED, NULL);

    row1 = create_footer_row();
    row2 = create_footer_row();

    s.mode_matrix = lv_buttonmatrix_create(row1);
    lv_buttonmatrix_set_map(s.mode_matrix, mode_map);
    lv_buttonmatrix_set_button_ctrl(s.mode_matrix, 0, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_button_ctrl(s.mode_matrix, 1, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(s.mode_matrix, true);
    lv_obj_set_size(s.mode_matrix, SCOPE_MODE_MATRIX_WIDTH, UI_SELECTOR_HEIGHT);
    lv_obj_set_style_pad_all(s.mode_matrix, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(s.mode_matrix, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s.mode_matrix, lv_color_hex(UI_COLOR_PANEL_ALT), LV_PART_MAIN);
    lv_obj_set_style_border_color(s.mode_matrix, lv_color_hex(UI_COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_border_width(s.mode_matrix, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(s.mode_matrix, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s.mode_matrix, lv_color_hex(UI_COLOR_PANEL_ALT), LV_PART_ITEMS);
    lv_obj_set_style_text_color(s.mode_matrix, lv_color_hex(UI_COLOR_TEXT), LV_PART_ITEMS);
    lv_obj_set_style_text_font(s.mode_matrix, UI_FONT_CONTROL, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(s.mode_matrix, lv_color_hex(UI_COLOR_ACCENT),
                              LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(s.mode_matrix, lv_color_hex(UI_COLOR_BG),
                                LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_radius(s.mode_matrix, 4, LV_PART_ITEMS);
    lv_obj_add_event_cb(s.mode_matrix, mode_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* set_hold() fills in the text. */
    s.run_button = ui_style_create_button(row1, "", &s.run_label);
    lv_obj_set_width(s.run_button, SCOPE_RUN_BUTTON_WIDTH);
    lv_obj_add_event_cb(s.run_button, run_event_cb, LV_EVENT_CLICKED, NULL);

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
    s.rate_tick = lv_tick_get();
    lv_obj_update_layout(s.chart);
    apply_geometry();
    clear_points();
    lv_buttonmatrix_set_button_ctrl(s.mode_matrix, (uint32_t)s.cfg.mode,
                                    LV_BUTTONMATRIX_CTRL_CHECKED);
    refresh_range(true);    /* no data yet: the default range for the unit */
    set_hold(false);

    s.timer = lv_timer_create(scope_tick, 1000U / SCOPE_REFRESH_HZ, NULL);
    return s.screen;
}
