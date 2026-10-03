/******************************************************************************
 * @file     prof.h
 * @brief    Code-placement profiling: PC-sampling histogram driven by a REFCLK
 *           compare interrupt, plus per-frame PMU counters around LVGL
 *           refreshes. Results live in the global g_prof (dump it with the
 *           debugger) and are printed over stdout when a capture finishes.
 *           Decode either form with tools/prof_report.py.
 ******************************************************************************/
#ifndef PROF_H
#define PROF_H

#include <stdint.h>

#ifndef PROF_ENABLE
#define PROF_ENABLE 0
#endif

#if PROF_ENABLE

/* Call from main() after refclk_cntr_init() and before the scheduler starts. */
void prof_init(void);

/* Call from the LVGL thread once the display exists (after port_disp_init()). */
void prof_attach_display(void);

/* Call from the FreeRTOS traceTASK_SWITCHED_IN hook with the incoming task name. */
void prof_task_switched_in(const char *name);

/* Discard the current results and start a new capture immediately. */
void prof_rearm(void);

#else

static inline void prof_init(void) {}
static inline void prof_attach_display(void) {}
static inline void prof_task_switched_in(const char *name) { (void)name; }
static inline void prof_rearm(void) {}

#endif

#endif // PROF_H
