/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#include <stdio.h>

#include <alif.h>
#include <drv_counter.h>
#include <pm_soc_clk.h>
#include <RTE_Components.h>

#include "se_services_port.h"
#include "board_config.h"
#include "board_utils.h"

#include "port_disp.h"
#include "prof.h"
#include "sensor_task.h"
#include "settings.h"
#include "ui.h"

#if defined(RTE_CMSIS_Compiler_STDIN) || defined(RTE_CMSIS_Compiler_STDOUT)
#include "retarget_init.h"
#include "retarget_config.h"
#endif

// LVGL tick timer period
const uint32_t LVGL_TICK_TIME_MS = 1;

#if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "timers.h"

// LVGL tick timer handle
TimerHandle_t lvgl_tick_timer_handle = NULL;
// LVGL task handle
TaskHandle_t lvgl_task_handle = NULL;
#endif

void clock_init()
{
    uint32_t error_code = 0;
    uint32_t service_error_code = 0;

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_CLK_100M, true, &service_error_code);
    if(error_code || service_error_code){
        printf("SE: 100MHz clock enable error_code=%u se_error_code=%u\n", error_code, service_error_code);
        return;
    }

    error_code = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSC, true, &service_error_code);
    if(error_code || service_error_code){
        printf("SE: HFOSC enable error_code=%u se_error_code=%u\n", error_code, service_error_code);
        return;
    }
}

uint32_t apply_run_profile()
{
    run_profile_t default_runprof;
    uint32_t service_error_code;
    uint32_t err;

    default_runprof.dcdc_mode       = DCDC_MODE_PWM;
    default_runprof.memory_blocks   = SERAM_MASK | SRAM0_MASK | SRAM1_MASK | MRAM_MASK | BACKUP4K_MASK;
    default_runprof.phy_pwr_gating  = LDO_PHY_MASK | MIPI_PLL_DPHY_MASK | MIPI_TX_DPHY_MASK;
    default_runprof.power_domains   = PD_VBAT_AON_MASK | PD_SSE700_AON_MASK | PD_SYST_MASK | PD_DBSS_MASK;
    default_runprof.dcdc_voltage    = 825;
    default_runprof.aon_clk_src     = CLK_SRC_LFXO;
    default_runprof.run_clk_src     = CLK_SRC_PLL;
    default_runprof.scaled_clk_freq = SCALED_FREQ_XO_HIGH_DIV_38_4_MHZ;
#if defined(M55_HE)
    default_runprof.cpu_clk_freq    = CLOCK_FREQUENCY_160MHZ;
#elif defined(M55_HP)
    default_runprof.cpu_clk_freq    = CLOCK_FREQUENCY_400MHZ;
#endif
    default_runprof.vdd_ioflex_3V3  = IOFLEX_LEVEL_3V3;
    return SERVICES_set_run_cfg(se_services_s_handle, &default_runprof, &service_error_code);
}

#if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
void lvgl_tick_timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    lv_tick_inc(LVGL_TICK_TIME_MS);
}
#else
#define SYSTICKS_PER_SECOND    1000
void SysTick_Handler(void)
{
    lv_tick_inc(LVGL_TICK_TIME_MS);
}
#endif

void lvgl_thread(void *pvParam)
{
    (void) pvParam;

    BOARD_LED1_Init();

    // Initialize display (and touch), then build the UI
    port_disp_init();
    prof_attach_display();
    ui_init();

    while (true) {
        uint32_t ms_till_next_run = lv_timer_handler();
        // Clamp to 1..10 ms so the chart timer never starves
        if (ms_till_next_run < 1) ms_till_next_run = 1;
        if (ms_till_next_run > 10) ms_till_next_run = 10;
        #if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
        const TickType_t xDelay = ms_till_next_run/portTICK_PERIOD_MS;
        vTaskDelay(xDelay);
        #else
        sys_busy_loop_us(ms_till_next_run*1000);
        #endif
        // BOARD_LED1_Control(BOARD_LED_STATE_TOGGLE);
    }
}

int main (void)
{
    se_services_port_init();
    apply_run_profile();

    board_gpios_config();
    board_pins_config();
    pm_core_clk_update();
    pm_soc_clk_get_busclk();
    refclk_cntr_init();

#if defined(RTE_CMSIS_Compiler_STDIN_Custom)
    stdin_init();
#endif
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    stdout_init();
#endif

    clock_init();

#if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
    prof_init();
    settings_init();

    // Sensor task: high priority so sampling is not disturbed by rendering
    const sensor_task_config_t sensor_config = {
        SENSOR_TASK_DEFAULT_PERIOD_MS,
        tskIDLE_PRIORITY + 3U
    };
    if (!sensor_task_start(INA228_I2C_ADDR, &sensor_config)) {
        __BKPT();
    }
#endif

#if defined(LV_USE_OS) && (LV_USE_OS == LV_OS_FREERTOS)
    // Create application main thread (2048 words = 8 KB stack)
    if (xTaskCreate(lvgl_thread, "LVGL", 2048, NULL,
                    tskIDLE_PRIORITY+1, &lvgl_task_handle) != pdPASS) {
        vTaskDelete(lvgl_task_handle);
        __BKPT();
    }

    // Create LVGL tick timer
    lvgl_tick_timer_handle = xTimerCreate("lvgl_tick",
                            LVGL_TICK_TIME_MS/portTICK_PERIOD_MS,
                            pdTRUE, NULL,
                            lvgl_tick_timer_callback);
    if (lvgl_tick_timer_handle == NULL) {
        __BKPT();
    }
    xTimerStart(lvgl_tick_timer_handle, 0);

    // Start thread execution
    vTaskStartScheduler();
#else
    // Initialize SysTick timer as LVGL tick timer
    SysTick_Config(SystemCoreClock/(LVGL_TICK_TIME_MS*SYSTICKS_PER_SECOND));
    // Call LVGL thread function directly
    lvgl_thread(NULL);
#endif

    return 0;
}
