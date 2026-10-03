/******************************************************************************
 * @file     prof.c
 * @brief    PC-sampling profiler and per-frame PMU counters (see prof.h)
 *
 * Sampler: REFCLK compare frame PROF_CNTBASE fires at about PROF_RATE_HZ with
 * +/-12.5% jitter, so it cannot lock onto the 1 kHz RTOS tick or the frame
 * rate. The handler reads the interrupted PC from the exception frame and
 * bumps a bucket in a histogram that covers every region code can run from:
 * MRAM (vector table up to the copy table) and each copy-table destination
 * outside DTCM (ITCM, SRAM1 code). The histogram lives in DTCM, which is not
 * cached, so recording samples does not disturb the caches being measured.
 *
 * Frame counter: the PMU counts cycles, I/D-cache refills and front/back-end
 * stall cycles from LV_EVENT_REFR_START to LV_EVENT_REFR_READY. Counting
 * pauses during flush waits (vsync) and whenever a task other than the LVGL
 * thread or its "swdraw" render thread runs, so only rendering is counted.
 *
 * A capture starts PROF_WARMUP_MS after prof_init() and lasts PROF_CAPTURE_MS.
 * Write g_prof.rearm = 1 from the debugger (or call prof_rearm()) to start
 * another one.
 ******************************************************************************/
#include "prof.h"

#if PROF_ENABLE

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <alif.h>
#include <drv_counter.h>

#include "FreeRTOS.h"
#include "task.h"
#include "lvgl.h"

#ifndef PROF_CNTBASE
#define PROF_CNTBASE        3           /* REFCLK compare frame 0..3; plain digit, it is token-pasted */
#endif
#ifndef PROF_RATE_HZ
#define PROF_RATE_HZ        10000u      /* 0: no PC sampling, frame counter only */
#endif
#ifndef PROF_WARMUP_MS
#define PROF_WARMUP_MS      10000u      /* from prof_init() to the first capture */
#endif
#ifndef PROF_CAPTURE_MS
#define PROF_CAPTURE_MS     30000u
#endif
#ifndef PROF_MAX_BUCKETS
#define PROF_MAX_BUCKETS    65536u      /* 256 KB of DTCM */
#endif
#ifndef PROF_UART_DUMP
#define PROF_UART_DUMP      1           /* print the capture over stdout when it finishes */
#endif

#define PROF_MIN_SHIFT      3u          /* 8-byte buckets, coarsened until the windows fit */
#define PROF_IDLE_HZ        10u         /* interrupt rate outside a capture */
#define PROF_MAX_WINDOWS    4u
#define PROF_MAX_WINDOW     0x00800000u /* sanity limit for one code region */
#define PROF_UNMAPPED_RING  8u
#define PROF_MAGIC          0x464F5250u /* "PROF" */
#define PROF_VERSION        1u

#define PROF_DTCM_BASE      0x20000000u
#define PROF_DTCM_END       0x30000000u

/* Intervals are measured with the low 32 bits of the REFCLK counter, which wrap every ~42 s at 100 MHz */
_Static_assert((PROF_WARMUP_MS <= 40000u) && (PROF_CAPTURE_MS <= 40000u), "interval must fit in 32-bit REFCLK ticks");

/* REFCLK compare frame registers (Arm generic timer CNTBaseN layout) */
#define PROF_CNTP_TVAL          (*(volatile uint32_t *)(0x1A230028u + (0x10000u * PROF_CNTBASE)))
#define PROF_CNTP_CTL           (*(volatile uint32_t *)(0x1A23002Cu + (0x10000u * PROF_CNTBASE)))
#define PROF_CNTP_CTL_ENABLE    (1u << 0)
#define PROF_CNTP_CTL_ISTATUS   (1u << 2)
#define PROF_IRQn               ((IRQn_Type)(67 + PROF_CNTBASE))

#define PROF_HANDLER_(n)        REFCLK_CNTBASE##n##_IRQHandler
#define PROF_HANDLER(n)         PROF_HANDLER_(n)

enum {
    PROF_STATE_WAIT     = 0,
    PROF_STATE_SAMPLING = 1,
    PROF_STATE_DONE     = 2,
};

typedef struct {
    uint32_t base;
    uint32_t size;
    uint32_t first_bucket;
    uint32_t n_buckets;
} prof_window_t;

typedef struct {
    uint32_t frames;
    uint32_t empty_frames;      /* refreshes that flushed nothing; not accumulated */
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint64_t cycles;
    uint64_t i_refill;
    uint64_t fe_stall;
    uint64_t d_refill;
    uint64_t be_stall;
} prof_frame_stats_t;

/* tools/prof_report.py decodes this layout (HEADER_FMT); keep the two in sync */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t header_size;
    volatile uint32_t state;
    volatile uint32_t gen;
    volatile uint32_t rearm;    /* write 1 from the debugger to restart */
    uint32_t rate_hz;
    uint32_t tick_hz;
    uint32_t cpu_hz;
    uint32_t shift;
    uint32_t n_windows;
    uint32_t n_buckets;
    uint32_t max_buckets;
    uint32_t samples;
    uint32_t unmapped;
    uint32_t start_tick;
    uint32_t elapsed_ticks;
    uint32_t capture_ticks;
    uint32_t isr_addr;          /* lets the host check that the ELF matches */
    uint32_t self_addr;
    uint32_t unmapped_pc[PROF_UNMAPPED_RING];
    prof_window_t win[PROF_MAX_WINDOWS];
    prof_frame_stats_t frame;
    uint32_t buckets[PROF_MAX_BUCKETS];
} prof_capture_t;

_Static_assert(offsetof(prof_capture_t, buckets) == 232u, "layout must match tools/prof_report.py");

/* In .bss, i.e. DTCM */
prof_capture_t g_prof;

typedef struct {
    const uint32_t *src;
    uint32_t       *dest;
    uint32_t        wlen;
} prof_copy_entry_t;

/* Linker-script symbols, bound by assembler name so they cannot clash with
 * the startup code's own declarations of the same symbols */
extern const prof_copy_entry_t prof_copy_table_start[] __asm__("__copy_table_start__");
extern const prof_copy_entry_t prof_copy_table_end[] __asm__("__copy_table_end__");

void prof_sample_isr(const uint32_t *frame);
void PROF_HANDLER(PROF_CNTBASE)(void);

static uint32_t s_init_tick;
static uint32_t s_warmup_ticks;
static uint32_t s_idle_period;
static uint32_t s_fast_base;
static uint32_t s_fast_jitter;
static uint32_t s_rng;          /* seeded in prof_init() so it stays in .bss (DTCM) */

/* Frame counter state; changed only with interrupts masked or from the scheduler */
static bool s_pmu_ready;
static volatile bool s_frame_active;
static volatile bool s_flush_wait;
static volatile bool s_task_is_lvgl;
static bool s_frame_flushed;
static bool s_frame_counted;
static uint32_t s_frame_gen;

/*----------------------------------------------------------------------------
 * PC sampler
 *--------------------------------------------------------------------------*/
static inline uint32_t fast_period(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_fast_base - (s_fast_jitter / 2u) + (s_rng % s_fast_jitter);
}

static void capture_start(uint32_t now)
{
    memset(g_prof.buckets, 0, g_prof.n_buckets * sizeof(g_prof.buckets[0]));
    memset(g_prof.unmapped_pc, 0, sizeof(g_prof.unmapped_pc));
    memset(&g_prof.frame, 0, sizeof(g_prof.frame));
    g_prof.frame.min_cycles = UINT32_MAX;
    g_prof.samples = 0u;
    g_prof.unmapped = 0u;
    g_prof.cpu_hz = SystemCoreClock;
    g_prof.start_tick = now;
    g_prof.elapsed_ticks = 0u;
    g_prof.gen++;
    g_prof.state = PROF_STATE_SAMPLING;
}

static inline void record(uint32_t pc)
{
    g_prof.samples++;
    for (uint32_t i = 0u; i < g_prof.n_windows; i++) {
        const prof_window_t *w = &g_prof.win[i];
        uint32_t off = pc - w->base;
        if (off < w->size) {
            g_prof.buckets[w->first_bucket + (off >> g_prof.shift)]++;
            return;
        }
    }
    g_prof.unmapped_pc[g_prof.unmapped % PROF_UNMAPPED_RING] = pc;
    g_prof.unmapped++;
}

/* Tail-called by the naked handler with the interrupted context's exception frame */
__attribute__((used)) void prof_sample_isr(const uint32_t *frame)
{
    /* Level interrupt: ignore a re-entry taken before the line dropped */
    if ((PROF_CNTP_CTL & PROF_CNTP_CTL_ISTATUS) == 0u) {
        return;
    }

    uint32_t now = refclk_cntr_val();
    uint32_t next = s_idle_period;

    if (g_prof.rearm != 0u) {
        g_prof.rearm = 0u;
        capture_start(now);
    } else if ((g_prof.state == PROF_STATE_WAIT) && ((now - s_init_tick) >= s_warmup_ticks)) {
        capture_start(now);
    }

    if (g_prof.state == PROF_STATE_SAMPLING) {
        g_prof.elapsed_ticks = now - g_prof.start_tick;
        if (g_prof.elapsed_ticks >= g_prof.capture_ticks) {
            g_prof.state = PROF_STATE_DONE;
        } else if (g_prof.rate_hz != 0u) {
            record(frame[6]);   /* stacked PC; same offset with or without an FP frame */
            next = fast_period();
        }
    }

    PROF_CNTP_TVAL = next;
    (void)PROF_CNTP_CTL;        /* read back so the write lands before exception return */
}

/* Pick MSP or PSP from EXC_RETURN bit 2 and tail-call prof_sample_isr(frame).
 * movw/movt + bx instead of b, so the call reaches wherever the linker puts it. */
__attribute__((naked)) void PROF_HANDLER(PROF_CNTBASE)(void)
{
    __asm volatile(
        "tst    lr, #4                          \n"
        "ite    eq                              \n"
        "mrseq  r0, msp                         \n"
        "mrsne  r0, psp                         \n"
        "movw   r1, #:lower16:prof_sample_isr   \n"
        "movt   r1, #:upper16:prof_sample_isr   \n"
        "bx     r1                              \n"
    );
}

static void add_window(uint32_t base, uint32_t size)
{
    if ((size == 0u) || (size > PROF_MAX_WINDOW) || (g_prof.n_windows >= PROF_MAX_WINDOWS)) {
        return;
    }
    g_prof.win[g_prof.n_windows].base = base;
    g_prof.win[g_prof.n_windows].size = size;
    g_prof.n_windows++;
}

static void setup_windows(void)
{
    /* MRAM code: .startup.at_mram runs from the vector table up to the copy table */
    uint32_t mram_base = SCB->VTOR;
    uint32_t mram_end = (uint32_t)(uintptr_t)prof_copy_table_start;
    if (mram_end > mram_base) {
        add_window(mram_base, mram_end - mram_base);
    }

    /* RAM code: every copy-table destination except DTCM (.data) */
    for (const prof_copy_entry_t *e = prof_copy_table_start; e < prof_copy_table_end; e++) {
        uint32_t dest = (uint32_t)(uintptr_t)e->dest;
        if ((dest >= PROF_DTCM_BASE) && (dest < PROF_DTCM_END)) {
            continue;
        }
        add_window(dest, e->wlen * 4u);
    }

    /* Coarsen the buckets until every window fits */
    uint32_t shift = PROF_MIN_SHIFT;
    uint32_t total;
    for (;;) {
        total = 0u;
        for (uint32_t i = 0u; i < g_prof.n_windows; i++) {
            total += (g_prof.win[i].size + (1u << shift) - 1u) >> shift;
        }
        if (total <= PROF_MAX_BUCKETS) {
            break;
        }
        shift++;
    }

    uint32_t first = 0u;
    for (uint32_t i = 0u; i < g_prof.n_windows; i++) {
        g_prof.win[i].first_bucket = first;
        g_prof.win[i].n_buckets = (g_prof.win[i].size + (1u << shift) - 1u) >> shift;
        first += g_prof.win[i].n_buckets;
    }
    g_prof.shift = shift;
    g_prof.n_buckets = total;
}

/*----------------------------------------------------------------------------
 * Frame counter
 *--------------------------------------------------------------------------*/
static void pmu_init(void)
{
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
    /* The application runs Secure; the PMU only counts Secure code when
     * secure non-invasive debug is enabled */
    DCB->DAUTHCTRL |= DCB_DAUTHCTRL_SPNIDENSEL_Msk | DCB_DAUTHCTRL_INTSPNIDEN_Msk;

    ARM_PMU_Disable();
    /* Event counters are 16-bit: chain each even counter into the next odd one */
    ARM_PMU_Set_EVTYPER(0u, ARM_PMU_L1I_CACHE_REFILL);
    ARM_PMU_Set_EVTYPER(1u, ARM_PMU_CHAIN);
    ARM_PMU_Set_EVTYPER(2u, ARM_PMU_STALL_FRONTEND);
    ARM_PMU_Set_EVTYPER(3u, ARM_PMU_CHAIN);
    ARM_PMU_Set_EVTYPER(4u, ARM_PMU_L1D_CACHE_REFILL);
    ARM_PMU_Set_EVTYPER(5u, ARM_PMU_CHAIN);
    ARM_PMU_Set_EVTYPER(6u, ARM_PMU_STALL_BACKEND);
    ARM_PMU_Set_EVTYPER(7u, ARM_PMU_CHAIN);
    ARM_PMU_CNTR_Enable(PMU_CNTENSET_CCNTR_ENABLE_Msk | 0xFFu);
    s_pmu_ready = true;
}

static inline uint32_t pmu_pair(uint32_t even)
{
    return (ARM_PMU_Get_EVCNTR(even + 1u) << 16) | ARM_PMU_Get_EVCNTR(even);
}

/* Count only inside a refresh, outside flush waits, while LVGL or swdraw runs */
static void pmu_gate(void)
{
    if (!s_pmu_ready) {
        return;
    }
    if (s_frame_active && !s_flush_wait && s_task_is_lvgl) {
        ARM_PMU_Enable();
    } else {
        ARM_PMU_Disable();
    }
}

void prof_task_switched_in(const char *name)
{
    s_task_is_lvgl = (strcmp(name, "LVGL") == 0) || (strcmp(name, "swdraw") == 0);
    pmu_gate();
}

static void frame_accumulate(void)
{
    prof_frame_stats_t *f = &g_prof.frame;
    uint32_t cycles = ARM_PMU_Get_CCNTR();

    f->frames++;
    f->cycles += cycles;
    f->i_refill += pmu_pair(0u);
    f->fe_stall += pmu_pair(2u);
    f->d_refill += pmu_pair(4u);
    f->be_stall += pmu_pair(6u);
    if (cycles < f->min_cycles) {
        f->min_cycles = cycles;
    }
    if (cycles > f->max_cycles) {
        f->max_cycles = cycles;
    }
}

static void frame_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    /* PRIMASK rather than a FreeRTOS critical section: the capture state is
     * also changed by the sampler, which runs above configMAX_SYSCALL */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    switch (code) {
    case LV_EVENT_REFR_START:
        ARM_PMU_Disable();
        ARM_PMU_CYCCNT_Reset();
        ARM_PMU_EVCNTR_ALL_Reset();
        s_frame_counted = (g_prof.state == PROF_STATE_SAMPLING);
        s_frame_gen = g_prof.gen;
        s_frame_flushed = false;
        s_flush_wait = false;
        s_task_is_lvgl = true;  /* display events run in the LVGL thread */
        s_frame_active = true;
        break;
    case LV_EVENT_FLUSH_START:
        s_frame_flushed = true;
        break;
    case LV_EVENT_FLUSH_WAIT_START:
        s_flush_wait = true;
        break;
    case LV_EVENT_FLUSH_WAIT_FINISH:
        s_flush_wait = false;
        break;
    case LV_EVENT_REFR_READY:
        s_frame_active = false;
        pmu_gate();
        /* Only frames that started and ended inside the same capture */
        if (s_frame_counted && (g_prof.state == PROF_STATE_SAMPLING) && (g_prof.gen == s_frame_gen)) {
            if (s_frame_flushed) {
                frame_accumulate();
            } else {
                g_prof.frame.empty_frames++;
            }
        }
        break;
    default:
        break;
    }

    pmu_gate();
    __set_PRIMASK(primask);
}

void prof_attach_display(void)
{
    static const lv_event_code_t codes[] = {
        LV_EVENT_REFR_START,
        LV_EVENT_FLUSH_START,
        LV_EVENT_FLUSH_WAIT_START,
        LV_EVENT_FLUSH_WAIT_FINISH,
        LV_EVENT_REFR_READY,
    };

    lv_display_t *disp = lv_display_get_default();
    if (disp == NULL) {
        return;
    }
    for (uint32_t i = 0u; i < sizeof(codes) / sizeof(codes[0]); i++) {
        lv_display_add_event_cb(disp, frame_event_cb, codes[i], NULL);
    }
}

/*----------------------------------------------------------------------------
 * UART dump
 *--------------------------------------------------------------------------*/
#if PROF_UART_DUMP
#define HEX(v)      ((unsigned long)(v))
#define HEX64(v)    HEX((v) >> 32), HEX((uint32_t)(v))

/* Line-oriented so tools/prof_report.py can pick it out of a mixed serial log */
static void dump_uart(void)
{
    const prof_frame_stats_t *f = &g_prof.frame;

    printf("PROF BEGIN %lx\n", HEX(PROF_VERSION));
    printf("PROF H gen=%lx rate_hz=%lx tick_hz=%lx cpu_hz=%lx shift=%lx n_windows=%lx n_buckets=%lx "
           "samples=%lx unmapped=%lx elapsed_ticks=%lx isr_addr=%lx self_addr=%lx\n",
           HEX(g_prof.gen), HEX(g_prof.rate_hz), HEX(g_prof.tick_hz), HEX(g_prof.cpu_hz),
           HEX(g_prof.shift), HEX(g_prof.n_windows), HEX(g_prof.n_buckets), HEX(g_prof.samples),
           HEX(g_prof.unmapped), HEX(g_prof.elapsed_ticks), HEX(g_prof.isr_addr), HEX(g_prof.self_addr));
    for (uint32_t i = 0u; i < g_prof.n_windows; i++) {
        printf("PROF W base=%lx size=%lx first=%lx n=%lx\n",
               HEX(g_prof.win[i].base), HEX(g_prof.win[i].size),
               HEX(g_prof.win[i].first_bucket), HEX(g_prof.win[i].n_buckets));
    }
    printf("PROF F frames=%lx empty_frames=%lx min_cycles=%lx max_cycles=%lx cycles=%lx%08lx "
           "i_refill=%lx%08lx fe_stall=%lx%08lx d_refill=%lx%08lx be_stall=%lx%08lx\n",
           HEX(f->frames), HEX(f->empty_frames), HEX(f->min_cycles), HEX(f->max_cycles),
           HEX64(f->cycles), HEX64(f->i_refill), HEX64(f->fe_stall), HEX64(f->d_refill), HEX64(f->be_stall));

    uint32_t n_pc = (g_prof.unmapped < PROF_UNMAPPED_RING) ? g_prof.unmapped : PROF_UNMAPPED_RING;
    printf("PROF U");
    for (uint32_t i = 0u; i < n_pc; i++) {
        printf(" %lx", HEX(g_prof.unmapped_pc[i]));
    }
    printf("\n");

    uint32_t nonzero = 0u;
    uint32_t sum = 0u;
    uint32_t on_line = 0u;
    for (uint32_t i = 0u; i < g_prof.n_buckets; i++) {
        uint32_t c = g_prof.buckets[i];
        if (c == 0u) {
            continue;
        }
        if (on_line == 0u) {
            printf("PROF B");
        }
        printf(" %lx %lx", HEX(i), HEX(c));
        if (++on_line == 8u) {
            printf("\n");
            on_line = 0u;
        }
        nonzero++;
        sum += c;
    }
    if (on_line != 0u) {
        printf("\n");
    }
    printf("PROF END nonzero=%lx sum=%lx\n", HEX(nonzero), HEX(sum));
}

static void dump_task(void *arg)
{
    (void)arg;
    uint32_t dumped_gen = 0u;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if ((g_prof.state == PROF_STATE_DONE) && (g_prof.gen != dumped_gen)) {
            dumped_gen = g_prof.gen;
            vTaskDelay(pdMS_TO_TICKS(100));     /* let an in-flight frame finish */
            dump_uart();
        }
    }
}
#endif

/*----------------------------------------------------------------------------
 * Public API
 *--------------------------------------------------------------------------*/
void prof_rearm(void)
{
    g_prof.rearm = 1u;
}

void prof_init(void)
{
    uint32_t tick_hz = refclk_cntr_freq();
    if (tick_hz == 0u) {
        tick_hz = 100000000u;
    }

    g_prof.magic = PROF_MAGIC;
    g_prof.version = PROF_VERSION;
    g_prof.header_size = (uint32_t)offsetof(prof_capture_t, buckets);
    g_prof.max_buckets = PROF_MAX_BUCKETS;
    g_prof.rate_hz = PROF_RATE_HZ;
    g_prof.tick_hz = tick_hz;
    g_prof.cpu_hz = SystemCoreClock;
    g_prof.capture_ticks = (uint32_t)(((uint64_t)tick_hz * PROF_CAPTURE_MS) / 1000u);
    g_prof.isr_addr = (uint32_t)(uintptr_t)&prof_sample_isr;
    g_prof.self_addr = (uint32_t)(uintptr_t)&g_prof;
    setup_windows();

    s_warmup_ticks = (uint32_t)(((uint64_t)tick_hz * PROF_WARMUP_MS) / 1000u);
    s_idle_period = tick_hz / PROF_IDLE_HZ;
    s_rng = 0x2545F491u ^ refclk_cntr_val();
    if (s_rng == 0u) {
        s_rng = 1u;             /* xorshift never leaves zero */
    }
    if (PROF_RATE_HZ != 0u) {
        s_fast_base = tick_hz / PROF_RATE_HZ;
        s_fast_jitter = (s_fast_base >= 4u) ? (s_fast_base / 4u) : 1u;
    }

    pmu_init();

    s_init_tick = refclk_cntr_val();
    refclk_cntr_enable_cntbase(PROF_CNTBASE);
    /* Highest priority, above configMAX_SYSCALL, so FreeRTOS critical sections
     * are sampled too; the handler makes no RTOS calls */
    NVIC_SetPriority(PROF_IRQn, 0u);
    NVIC_ClearPendingIRQ(PROF_IRQn);
    PROF_CNTP_TVAL = s_idle_period;
    PROF_CNTP_CTL = PROF_CNTP_CTL_ENABLE;
    NVIC_EnableIRQ(PROF_IRQn);

#if PROF_UART_DUMP
    xTaskCreate(dump_task, "prof", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
#endif
}

#endif // PROF_ENABLE
