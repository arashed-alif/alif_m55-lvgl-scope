/**
 * @file lv_port_disp.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "port_disp.h"
#include <stdbool.h>
#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include <RTE_Device.h>
#include "Driver_CDC200.h" // Display driver
#include "Driver_Touch_Screen.h"
#include "motion_task.h"
#include "sys_utils.h"     // ENABLE, DISABLE macros

#if LV_USE_DRAW_DAVE2D
#include "dave_cfg.h"
#include "dave_d0lib.h"
#include "dave_driver.h"
#include "lv_draw_dave2d_utils.h"
#endif

#if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#define USE_EVT_GROUP
#endif // LV_OS_FREERTOS

/*********************
 *      DEFINES
 *********************/
#ifndef MY_DISP_HOR_RES
    // Replace the macro MY_DISP_HOR_RES with the actual screen width.
    #define MY_DISP_HOR_RES    (RTE_PANEL_HACTIVE_TIME)
#endif

#ifndef MY_DISP_VER_RES
    // Replace the macro MY_DISP_HOR_RES with the actual screen height.
    #define MY_DISP_VER_RES    (RTE_PANEL_VACTIVE_LINE)
#endif

#if ((LV_COLOR_DEPTH == 16) && (RTE_CDC200_PIXEL_FORMAT != 2)) || \
    ((LV_COLOR_DEPTH == 24) && (RTE_CDC200_PIXEL_FORMAT != 1)) || \
    ((LV_COLOR_DEPTH == 32) && (RTE_CDC200_PIXEL_FORMAT != 0))
#error "The LV_COLOR_DEPTH and RTE_CDC200_PIXEL_FORMAT must match."
#endif

#if defined(USE_EVT_GROUP)
#define EVENT_DISP_BUFFER_READY     ( 1 << 0 )
#define EVENT_DISP_BUFFER_CHANGED   ( 1 << 1 )
#endif

#if LV_USE_DRAW_DAVE2D
#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
// D/AVE D0 heap address and size
#define D1_HEAP_SIZE	0x80000
#endif
#endif

/**********************
 *      TYPEDEFS
 **********************/

#pragma pack(1)
#if RTE_CDC200_PIXEL_FORMAT == 0    // ARGB8888
typedef lv_color32_t Pixel;
#elif RTE_CDC200_PIXEL_FORMAT == 1  // RGB888
typedef lv_color_t Pixel;
#elif RTE_CDC200_PIXEL_FORMAT == 2  // RGB565
typedef lv_color16_t Pixel;
#else
#error "CDC200 Unsupported color format"
#endif
#pragma pack()

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);

static void disp_flush_wait(lv_display_t * disp);

static void disp_callback(uint32_t event);

static void touch_init(void);

static void touch_read(lv_indev_t * indev, lv_indev_data_t * data);

/**********************
 *  STATIC VARIABLES
 **********************/

#if LV_USE_DRAW_DAVE2D
#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
static uint8_t __attribute__((section(".bss.at_sram1"))) d0_heap[D1_HEAP_SIZE];
#endif
#endif

static Pixel lcd_buffer_1[MY_DISP_VER_RES][MY_DISP_HOR_RES]
            __attribute__((section(".bss.lcd_frame_buf"))) = {0};
static Pixel lcd_buffer_2[MY_DISP_VER_RES][MY_DISP_HOR_RES]
            __attribute__((section(".bss.lcd_frame_buf"))) = {0};

extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

extern ARM_DRIVER_TOUCH_SCREEN GT911;
static ARM_DRIVER_TOUCH_SCREEN *Touchdrv = &GT911;

static volatile bool disp_flush_enabled = true;
static ARM_TOUCH_STATE touch_state;

#if defined(USE_EVT_GROUP)
static EventGroupHandle_t dispEventGroupHandle = NULL;
#else
static volatile bool disp_buf_ready = true;
static volatile bool disp_buf_changed = false;
#endif

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void port_disp_init(void)
{
    #if defined(USE_EVT_GROUP)
    // Create event group to sync display states
    dispEventGroupHandle = xEventGroupCreate();
    #endif

    /*-------------------------
     * Initialize your display & touch panel
     * -----------------------*/
    disp_init();

    touch_init();

#if LV_USE_DRAW_DAVE2D
#if (D1_MEM_ALLOC == D1_MALLOC_D0LIB)
    /*-------------------------
     * Initialize D/AVE D0 heap
     * -----------------------*/
    if (!d0_initheapmanager(d0_heap, sizeof(d0_heap), d0_mm_fixed_range,
                            NULL, 0, 0, 0, d0_ma_unified))
    {
        printf("\r\nError: Heap manager initialization failed\n");
        return;
    }
#endif
#endif
    lv_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/
    lv_display_t * disp = lv_display_create(MY_DISP_HOR_RES, MY_DISP_VER_RES);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_flush_wait_cb(disp, disp_flush_wait);

    lv_indev_t * touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read);

    /* Two buffers screen sized buffer for double buffering.
     * Both LV_DISPLAY_RENDER_MODE_DIRECT and LV_DISPLAY_RENDER_MODE_FULL works, see their comments*/
    lv_display_set_buffers(disp, lcd_buffer_1, lcd_buffer_2,
                           sizeof(lcd_buffer_1),
                           LV_DISPLAY_RENDER_MODE_DIRECT);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /* Initialize CDC driver */
    int ret = CDCdrv->Initialize(disp_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC init failed\n");
        __BKPT(0);
        return;
    }

    /* Power control CDC */
    ret = CDCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC Power up failed\n");
        __BKPT(0);
        return;
    }

    /* configure CDC controller */
    ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)lcd_buffer_1);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC controller configuration failed\n");
        __BKPT(0);
        return;
    }

    /* Enable CDC SCANLINE0 event */
    ret = CDCdrv->Control(CDC200_SCANLINE0_EVENT, ENABLE);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC200_SCANLINE0_EVENT enable failed\n");
        __BKPT(0);
        return;
    }

#if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
    /*-------------------------
     * Set display interrupt priority to FreeRTOS kernel level
     * -----------------------*/
    NVIC_SetPriority(CDC_SCANLINE0_IRQ_IRQn, configKERNEL_INTERRUPT_PRIORITY);
#endif

    /* Start CDC */
    ret = CDCdrv->Start();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: CDC Start failed\n");
        __BKPT(0);
        return;
    }
}

static void touch_init(void)
{
    int ret = Touchdrv->Initialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Touch init failed\n");
        __BKPT(0);
        return;
    }

    ret = Touchdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: Touch Power up failed\n");
        __BKPT(0);
        return;
    }
}

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map)
{
    if (disp_flush_enabled && lv_disp_flush_is_last(disp_drv)) {

        #if defined(USE_EVT_GROUP)
        xEventGroupClearBits(dispEventGroupHandle, EVENT_DISP_BUFFER_READY);
        #else
        disp_buf_ready = false;
        #endif

        CDCdrv->Control(CDC200_FRAMEBUF_UPDATE_VSYNC, (uint32_t)px_map);

        #if defined(USE_EVT_GROUP)
        xEventGroupSetBits(dispEventGroupHandle, EVENT_DISP_BUFFER_CHANGED);
        #else
        disp_buf_changed = true;
        #endif
    }
}

/* Display buffer flush waiting callback */
static void disp_flush_wait(lv_display_t * disp)
{
    (void)disp;

#if defined(USE_EVT_GROUP)
    xEventGroupWaitBits(dispEventGroupHandle, EVENT_DISP_BUFFER_READY,
                        pdFALSE, pdFALSE, portMAX_DELAY);

#else
    while (!disp_buf_ready) {
        sys_busy_loop_us(3300);
    }
#endif
}

/* Read the latest touch state and expose the first contact to LVGL. A touch
 * that lands while the backlight is dimmed only wakes it: the whole press is
 * reported as released, so nothing on the (possibly dark) screen is hit. */
static void touch_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    static bool touch_throw_away;
    (void) indev;

    if (Touchdrv->GetState(&touch_state) != ARM_DRIVER_OK ||
        touch_state.numtouches == 0) {
        touch_throw_away = false;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    data->point.x = touch_state.coordinates[0].x;
    data->point.y = touch_state.coordinates[0].y;
    if (motion_task_touch()) {
        touch_throw_away = true;
    }
    data->state = touch_throw_away ? LV_INDEV_STATE_RELEASED : LV_INDEV_STATE_PRESSED;
}

/* Display event handler */
static void disp_callback(uint32_t event)
{
    if (event & ARM_CDC_SCANLINE0_EVENT) {
        #if defined(USE_EVT_GROUP)
        if (xEventGroupGetBitsFromISR(dispEventGroupHandle)
           & EVENT_DISP_BUFFER_CHANGED) {
            xEventGroupClearBitsFromISR(
                dispEventGroupHandle,
                EVENT_DISP_BUFFER_CHANGED);

            BaseType_t context_switch = pdFALSE;
            xEventGroupSetBitsFromISR(
                dispEventGroupHandle,
                EVENT_DISP_BUFFER_READY,
                &context_switch);
            portYIELD_FROM_ISR(context_switch);
        }
        #else
        if (disp_buf_changed) {
            disp_buf_changed = false;
            disp_buf_ready = true;
        }
        #endif
    }

    if (event & ARM_CDC_DSI_ERROR_EVENT) {
        // Transfer Error: Received Hardware error.
        __BKPT(0);
    }
}
