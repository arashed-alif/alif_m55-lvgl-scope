# Power Scope — Implementation Spec

An oscilloscope-style power/current monitor built on LVGL 9.5 + FreeRTOS 11.2, reading a TI INA228 and rendering on an 480×800 touch display. Target: Alif Ensemble MCU.

---

## 0. Project environment and what already exists

**Environment**
- Alif Ensemble MCU, CMSIS-Toolbox (`csolution` / `cproject`) project in VS Code.
- Build from the command line with `cbuild` (e.g. `cbuild <solution>.csolution.yml --context <project>.<build>+<target>`; inspect the existing `.csolution.yml` for the exact context names). Fix all warnings you introduce.
- Debug with GDB + J-Link (`JLinkGDBServer` + `arm-none-eabi-gdb`). Use this to verify task stack high-water marks and to catch hard faults; don't rely on printf alone.
- The user works through the VS Code agent sidebar and can also build/debug from VS Code. Keep changes to the CMSIS project files (`.cproject.yml`) explicit and minimal — add new source files and include paths there, don't restructure the solution.

**Already done — do not rewrite**
- **INA228 driver.** Treat it as a black box. Bus address, I2C speed, shunt, calibration are all handled. Locate its header and use its existing APIs for reading bus voltage, current, and power. `TODO (agent): record the exact function names/units here after reading the header.` The ALERT pin is not wired; polling is the default. If a conversion-ready interrupt would meaningfully improve sample timing, note it in the deliverables as an optional hardware change rather than assuming it.
- **Display driver** with double buffering and LVGL flush integration. The LVGL benchmark demo already runs — use that as the reference for how LVGL is initialised, ticked, and driven in this project. Reuse it; don't create a second init path.
- **FreeRTOS** is integrated and working.

**Needs work**
- **Touch driver** exists but is old and no longer compiles. Update it to build against the current SDK/CMSIS driver APIs and LVGL 9.5's `lv_indev` API (`lv_indev_create()`, `lv_indev_set_type(LV_INDEV_TYPE_POINTER)`, `lv_indev_set_read_cb()`). This is milestone 1.

---

## 1. Goals

- Live, scrolling line graph of **either** current (mA) **or** power (mW) over time. One trace at a time; a toggle switches which is shown.
- Bus voltage shown as a numeric readout in a corner of the screen (not graphed).
- Touch-driven UI. Two screens: **Scope** (main) and **Settings**.
- Smooth rendering (≥ 20 fps chart refresh) without dropping sensor samples.
- Clean separation: existing INA228 driver → sample store → UI. No LVGL calls from the sensor task.

Non-goals: logging to storage, multiple traces, triggering, network export. Leave hooks only where cheap.

---

## 2. Architecture

### 2.1 FreeRTOS tasks

| Task | Priority | Period | Responsibility |
|---|---|---|---|
| `sensor_task` | High | Configurable, default 10 ms (100 Hz) | Call the existing INA228 read APIs for VBUS, current, power; convert to mA/mW; push to ring buffer. Use `vTaskDelayUntil()` for a stable period. |
| `lvgl_task` | Medium | Loop calling `lv_timer_handler()` and delaying by its return value (clamped 1–10 ms) | Owns all LVGL calls. Reuse whatever the benchmark demo already does here. |
| `ui_update` (LVGL timer, not a task) | — | Configurable, default 33 ms | Drains new samples from the ring buffer, decimates, updates chart + readouts. |

LVGL tick: use whatever the benchmark integration already provides; if none, `lv_tick_set_cb()` returning FreeRTOS ticks in ms.

LVGL threading: set `LV_USE_OS LV_OS_FREERTOS` in `lv_conf.h` so `lv_lock()`/`lv_unlock()` exist. The sensor task never touches LVGL. Stack sizes: `lvgl_task` ≥ 8 KB, `sensor_task` ≥ 2 KB; confirm with `uxTaskGetStackHighWaterMark()` under the debugger.

### 2.2 Data flow

```
INA228 driver (existing) --> sensor_task --> sample_ring (SPSC) --> ui_update timer --> lv_chart + labels
                                                  ^
                                   settings_t (mode, window, rate) — mutex-protected
```

### 2.3 Sample store

- Single-producer / single-consumer lock-free ring buffer (head/tail atomics). Producer = sensor task, consumer = UI timer.
- Each sample stores **both** current and power so the mA/mW toggle is a display change with no data loss:
  ```c
  typedef struct {
      uint32_t t_ms;
      float    vbus_v;
      float    current_ma;
      float    power_mw;
  } sample_t;
  ```
- Capacity: longest window × fastest rate, rounded to a power of two. 60 s × 500 Hz = 30000 → 32768 × 16 B = 512 KB. The Ensemble has plenty of SRAM but check which region the buffer lands in and that it's not competing with the display buffers; if needed cap window×rate (e.g. 60 s only at ≤ 100 Hz) and document the limit. Place the buffer in a suitable memory section via the project's linker attributes.

---

## 3. UI

Screen 480×800. LVGL 9.5 APIs (`lv_chart`, `lv_scale`, `lv_screen_load_anim`). Dark theme, one accent colour for the trace, a fixed-width-friendly font for numeric readouts (check which `lv_font_montserrat_*` sizes are enabled in `lv_conf.h`; enable 16/20/28 if not).

### 3.1 Scope screen (default)

```
┌───────────────────────────────────────────────────────────────┐
│ [≡ Settings]     POWER SCOPE          VBUS 12.034 V  ● RUN    │  48 px header
├────┬──────────────────────────────────────────────────────────┤
│ Y  │                                                           │
│ sc │                    lv_chart (line)                        │
│ al │                                                           │  ~380 px
│ e  │                                                           │
├────┴──────────────────────────────────────────────────────────┤
│ [ mA | mW ]  [ RUN / HOLD ]  1.2345 mW   1 s/div   avg 850 mW  pk 1.21 W │  52 px footer
└───────────────────────────────────────────────────────────────┘
```

- **Chart**: `LV_CHART_TYPE_LINE`, one series. Point count = chart pixel width (≈ 720). Own a static `int32_t` array, attach with `lv_chart_set_ext_y_array()`, rewrite it each UI tick, call `lv_chart_refresh()`. Hide point markers, line width 2. Division lines 10 × 8, subtle colour.
- **Y scale**: vertical `lv_scale` on the left, range bound to the chart's current Y range; update labels when the range changes. Chart values are `int32_t` — scale floats by a per-mode multiplier (×10 or ×100) and undo it in the scale labels.
- **X axis**: "N s/div" text in the footer.
- **VBUS readout**: top-right label, `%.3f V`, updated every UI tick even in HOLD.
- **Mode toggle** mA / mW: `lv_buttonmatrix` with `LV_BUTTONMATRIX_CTRL_CHECKABLE` + one-checked. Switching: change data source, unit suffix everywhere, Y range, and refill the chart from the ring.
- **RUN/HOLD**: freezes the chart only; the sensor task keeps filling the ring. On resume, refill so the display jumps to "now".
- **Live / avg / peak** labels in the footer. Peak resets on mode change, window change, or long-press on the label.
- **Cursor** (milestone 5): tap/drag on the chart shows a vertical cursor (`lv_chart_add_cursor`) with a floating label showing value and relative time; tap outside the chart hides it.

Auto-ranging Y (default): min/max over the visible window + 10 % headroom, rounded to a 1/2/5×10ⁿ step; only re-range if the new range differs by > 5 % (anti-flicker). Manual range from Settings overrides.

Decimation: when window samples > chart points, each point covers `k` samples; plot the **mean** (Settings option: **max**, to preserve spikes). When window samples < chart points, plot 1:1 and reduce the chart's point count to match.

### 3.2 Settings screen

Reached via the header button; return via back button (top-left) or swipe-right. `lv_screen_load_anim(LV_SCR_LOAD_ANIM_MOVE_LEFT, 200)` and the reverse. Scrollable flex column, one row per setting.

| Setting | Control | Values | Default |
|---|---|---|---|
| Display mode | `lv_buttonmatrix` | mA / mW | mW |
| Time window | `lv_dropdown` | 1, 2, 5, 10, 30, 60 s | 10 s |
| Sample rate | `lv_dropdown` | 10, 50, 100, 200, 500 Hz | 100 Hz |
| Y range | `lv_dropdown` | Auto / Manual | Auto |
| Y min / Y max (manual only) | `lv_textarea` + numeric `lv_keyboard` | float | 0 / 1000 |
| Decimation | `lv_dropdown` | Mean / Max | Mean |
| Chart refresh | `lv_dropdown` | 15 / 30 / 60 Hz | 30 Hz |
| Screen brightness (only if a PWM backlight exists) | `lv_slider` | 10–100 % | 100 % |
| Restore defaults | `lv_button` + confirm `lv_msgbox` | — | — |

Sample-rate caveat: the achievable rate is bounded by the INA228 conversion time + averaging configured inside the existing driver. If the driver exposes an averaging/conversion-time setter, add a "Filter: Fast / Normal / Smooth" dropdown that maps to it; otherwise omit it and clamp the rate options to what the driver can actually deliver (measure it).

Behaviour:
- Changes apply immediately, written to `settings_t`. The sensor task re-reads the rate on its next iteration.
- Changing window or sample rate clears and refills the chart (not the ring).
- Persistence is optional. If the Ensemble project already has a flash/NVM helper, debounce-save 2 s after the last change with magic + version + CRC validation on load; otherwise stub `settings_load()/settings_save()` and leave a TODO.

### 3.3 Touch behaviour

| Gesture | Where | Action |
|---|---|---|
| Tap | mA/mW buttons | switch mode |
| Tap | RUN/HOLD | toggle freeze |
| Tap / drag | chart | show/move cursor |
| Tap | empty area outside chart | hide cursor |
| Long press | peak label | reset peak |
| Swipe left / tap gear | scope screen | open settings |
| Swipe right / tap back | settings | return to scope |

---

## 4. Project structure (add to the existing cproject)

```
app/
  settings.c/.h       // settings_t, defaults, load/save, mutex
  sample_ring.c/.h    // SPSC ring buffer
  sensor_task.c/.h    // wraps existing INA228 driver
  ui/
    ui.c/.h           // ui_init(), screen switching, theme
    ui_scope.c/.h     // scope screen + update timer
    ui_settings.c/.h  // settings screen
    ui_style.c/.h     // colours, fonts, shared styles
drivers/
  touch/              // updated touch driver + lv_indev glue
lv_conf.h             // LV_USE_OS=LV_OS_FREERTOS; enable chart, scale, dropdown, keyboard,
                      // textarea, msgbox, buttonmatrix; disable the benchmark demo in the final build
```

Keep a build configuration (or a `-DUI_DEMO_DATA=1` define) that swaps the sensor task for a synthetic generator, so the UI can be iterated without the INA228 attached.

---

## 5. Milestones & acceptance criteria

1. **Touch driver** — compiles against the current SDK, registered as an LVGL pointer indev. *Accept:* run the LVGL widgets demo (`lv_demo_widgets`) and interact with buttons/sliders by touch; log raw coordinates to confirm orientation/calibration matches the display.
2. **Sensor task + ring buffer** — sensor task calls the existing INA228 API at the configured rate; ring never overruns at 500 Hz. *Accept:* dropped-sample counter stays 0 over 10 minutes; values match a bench meter within 2 %.
3. **Scope screen, static** — layout renders; VBUS label updates; chart shows a synthetic sine with `UI_DEMO_DATA=1`. *Accept:* ≥ 20 fps, no tearing (double buffering already in place).
4. **Live chart** — real data, auto-range, mA/mW toggle, RUN/HOLD. *Accept:* toggling mode swaps units everywhere and the trace refills instantly; HOLD freezes trace while VBUS keeps updating.
5. **Settings screen** — all controls wired, live effect, navigation both ways. *Accept:* changing window 10 s → 1 s visibly stretches the trace within one refresh.
6. **Polish** — cursor, peak reset, persistence (if available), restore defaults, stack high-water marks checked under GDB, benchmark demo removed from the final build.

---

## 6. Notes for the agent

- Read the existing INA228 header and the benchmark demo's LVGL init first; build on both.
- Never call LVGL from `sensor_task` or an ISR.
- Bound the UI timer's work: drain at most one window's worth of samples per tick; after a long HOLD, skip to the newest window.
- Make all hard-coded dimensions `#define`s at the top of `ui_scope.c`.
- Build with `cbuild` after every milestone and confirm zero new warnings; flash and verify on hardware with GDB + J-Link before declaring a milestone done.
