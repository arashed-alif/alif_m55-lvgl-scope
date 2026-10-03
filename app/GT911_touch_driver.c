/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     GT911_touch_driver.c
 * @author   Prasanna Ravi, Ahmad Rashed and Chandra Bhushan Singh
 * @email    prasanna.ravi@alifsemi.com , ahmad.rashed@alifsemi.com and
 *           chandrabhushan.singh@alifsemi.com
 * @version  V1.1.0
 * @date     3 August 2023
 * @brief    GT911 touch screen driver.
 * @bug      None.
 * @Note     App-local copy of the Ensemble 2.2.0 pack driver, reworked for
 *           FreeRTOS: I2C completion is signalled through a semaphore, so
 *           Initialize, PowerControl and GetState must be called from a task
 *           after the scheduler has started.
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include "sys_utils.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* GPIO Driver */
#include "Driver_IO.h"

/* I2C Driver */
#include "Driver_I2C.h"
#include "Driver_I2C_EX.h"

#include "board_config.h"

/* Touchscreen Driver */
#include "Driver_Touch_Screen.h"

#if !(RTE_GT911)
#error "GT911 Touch is not enabled in the RTE_Device.h"
#endif

#if !(RTE_TOUCH_SCREEN)
#error "Touch Driver not enabled in the RTE_Device.h"
#endif

#if !(RTE_ACTIVE_TOUCH_POINTS)
#error "GT911 at least enable one active touch point"
#endif

/* The pack driver could also run over the I3C controller in I2C mode; here the
 * I3C bus belongs to the INA228 driver, so only the plain I2C instances apply. */
#if defined(RTE_GT911_TOUCH_I2C_INSTANCE) && (RTE_GT911_TOUCH_I2C_INSTANCE == 4)
#error "GT911 over I3C is not supported by this driver (I3C is used by the INA228)"
#endif

/* Touch reset pin control GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(BOARD_TOUCH_RESET_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_RST = &ARM_Driver_GPIO_(BOARD_TOUCH_RESET_GPIO_PORT);

/* Touch INT pin control GPIO port */
extern ARM_DRIVER_GPIO  ARM_Driver_GPIO_(BOARD_TOUCH_INT_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIO_Driver_INT = &ARM_Driver_GPIO_(BOARD_TOUCH_INT_GPIO_PORT);

/* I2C Driver instance for Touch screen communication */
extern ARM_DRIVER_I2C  ARM_Driver_I2C_(BOARD_GT911_TOUCHSCREEN_I2C_INSTANCE);
static ARM_DRIVER_I2C *I2C_Driver = &ARM_Driver_I2C_(BOARD_GT911_TOUCHSCREEN_I2C_INSTANCE);

#define GT911_I2C_IRQN_(n)          I2C##n##_IRQ_IRQn
#define GT911_I2C_IRQN(n)           GT911_I2C_IRQN_(n)

/* GT911 Driver specific macros */

/* Set the GT911_SLAVE_ADDR macro to either 0x5D or 0x14 to select the desired I2C slave address */
#define GT911_SLAVE_ADDR            RTE_GT911_TOUCH_I2C_SLAVE_ADDRESS_SEL

/* Registers */
#define GT911_COMMAND_REG           0x8040
#define GT911_CONFIG_REG            0x8047  /* config version, first of the config block */
#define GT911_PRODUCT_ID            0x8140  /* "911\0", then firmware version (LE) */
#define GT911_TOUCH_STATUS          0x814E  /* status, followed by the touch point records */

/* Commands written to GT911_COMMAND_REG */
#define GT911_CMD_READ_COORD        0x00
#define GT911_CMD_SCREEN_OFF        0x05

/* GT911_TOUCH_STATUS bits */
#define GT911_STATUS_BUF_READY      0x80
#define GT911_NUM_TOUCH_POINTS_MSK  0x0F
#define GT911_MAX_TOUCH_POINTS      5

/* Touch point record: track id, X (LE), Y (LE), size (LE), reserved */
#define GT911_POINT_SIZE            8
#define GT911_POINT_X               1
#define GT911_POINT_Y               3

/* Config block offsets relative to GT911_CONFIG_REG */
#define GT911_CFG_VERSION           0
#define GT911_CFG_X_MAX             1   /* LE */
#define GT911_CFG_Y_MAX             3   /* LE */
#define GT911_CFG_MODULE_SWITCH1    6   /* bits 1:0 INT trigger mode */
#define GT911_CFG_REFRESH_RATE      15  /* bits 3:0 N, report period 5 + N ms */
#define GT911_CFG_READ_SIZE         16

#define GT911_INT_TRIGGER_MSK       0x03
#define GT911_INT_TRIGGER_RISING    0
#define GT911_INT_TRIGGER_FALLING   1
#define GT911_INT_TRIGGER_LOW       2
#define GT911_INT_TRIGGER_HIGH      3

#define GT911_REG_INDX_SIZE         2
#define GT911_MAX_WRITE_SIZE        4
#define TOUCH_I2C_TIMEOUT_MS        20

/* Power-on / reset timing, GT911 datasheet */
#define GT911_RESET_LOW_MS          20  /* T2 > 10 ms */
#define GT911_ADDR_SETUP_US         200 /* T3 > 100 us */
#define GT911_RESET_HIGH_MS         10  /* T4 > 5 ms */
#define GT911_INT_SYNC_MS           50  /* T5: INT held low before it becomes an input */
#define GT911_SLEEP_INT_LOW_MS      5

/* A held finger produces a report every 5 + N ms. With no report for this many
 * periods the release report was missed and the status register is polled. */
#define GT911_RELEASE_PERIODS       4

/* GT911 Driver status */
#define GT911_FLAG_DRV_INIT_DONE    (1U << 0)
#define GT911_FLAG_POWER_ENABLED    (1U << 1)

/* GT911 Touch event variables, written from ISRs */
static volatile bool     touch_event_gpio;
static volatile uint32_t touch_event_i2c;

/* GT911 Touch driver Info variable */
static struct GT911_DRV_INFO {
    uint32_t          touch_drv_status;
    SemaphoreHandle_t i2c_done;         /* given by the I2C callback */
    uint32_t          int_irq_arg;      /* GPIO IRQ config matching the panel's INT mode */
    ARM_TOUCH_STATE   last;             /* last report, returned until a new one arrives */
    TickType_t        armed_tick;       /* when the INT interrupt was last re-enabled */
    TickType_t        release_timeout;
    bool              poll_again;       /* re-read status on the next GetState */
} gt911_drv_info;

#define ARM_TOUCH_SCREEN_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 1) /*Driver version*/

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion                 = {ARM_TOUCH_SCREEN_API_VERSION,
                                                                 ARM_TOUCH_SCREEN_DRV_VERSION};

/* Driver Capabilities */
static const ARM_TOUCH_SCREEN_CAPABILITIES DriverCapabilities = {
    0,                               /* Not supports reentrant_operation */
    (RTE_ACTIVE_TOUCH_POINTS > 1),   /* Multiple touch points supported*/
    0                                /* reserved (must be zero)*/
};

/**
  \fn          ARM_DRIVER_VERSION ARM_TOUCH_SCREEN_GetVersion (void)
  \brief       Get touch screen driver version.
  \return      \ref ARM_DRIVER_VERSION.
*/
static ARM_DRIVER_VERSION ARM_TOUCH_SCREEN_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_TOUCH_SCREEN_CAPABILITIES ARM_TOUCH_SCREEN_GetCapabilities (void)
  \brief       Get touch screen driver capabilities.
  \return      \ref ARM_TOUCH_SCREEN_CAPABILITIES.
*/
static ARM_TOUCH_SCREEN_CAPABILITIES ARM_TOUCH_SCREEN_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn           void TOUCH_DelayMs(uint32_t ms)
  \brief        Block the calling task for at least ms milliseconds.
  \param[in]    ms : delay in milliseconds.
  */
static void TOUCH_DelayMs(uint32_t ms)
{
    /* +1: vTaskDelay(n) may return up to one tick early */
    vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
}

/**
  \fn           void TOUCH_I2C_CB(uint32_t event)
  \brief        GT911 Touch screen I2C callback event, runs in the I2C ISR.
  \param[in]    event: \ref I2C Event.
  */
static void TOUCH_I2C_CB(uint32_t event)
{
    BaseType_t woken = pdFALSE;

    touch_event_i2c = event;
    if (gt911_drv_info.i2c_done != NULL) {
        (void) xSemaphoreGiveFromISR(gt911_drv_info.i2c_done, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/**
  \fn           int32_t TOUCH_WaitI2C(void)
  \brief        Wait for the transfer started by the caller to finish.
                A timed-out transfer is aborted so the driver is not left busy.
  \return       \ref execution_status.
  */
static int32_t TOUCH_WaitI2C(void)
{
    uint32_t event;

    if (xSemaphoreTake(gt911_drv_info.i2c_done, pdMS_TO_TICKS(TOUCH_I2C_TIMEOUT_MS) + 1U) !=
        pdTRUE) {
        (void) I2C_Driver->Control(ARM_I2C_ABORT_TRANSFER, 0);
        return ARM_DRIVER_ERROR_TIMEOUT;
    }

    event = touch_event_i2c;

    /* A master transfer succeeded only if TRANSFER_DONE is the sole event */
    if (event != ARM_I2C_EVENT_TRANSFER_DONE) {
        if (event & ARM_I2C_EVENT_SDA_STUCK_LOW) {
            (void) I2C_Driver->Control(ARM_I2C_BUS_CLEAR, 0);
        }
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t TOUCH_Write(uint16_t reg_addr, const uint8_t *reg_data, uint8_t num_bytes)
  \brief        write register value to GT911 Touch screen registers.
  \param[in]    reg_addr : Register address.
  \param[in]    reg_data : pointer to register data.
  \param[in]    num_bytes : Number of bytes to write, at most GT911_MAX_WRITE_SIZE.
  \return       \ref execution_status.
  */
static int32_t TOUCH_Write(uint16_t reg_addr, const uint8_t *reg_data, uint8_t num_bytes)
{
    uint8_t tx_data[GT911_REG_INDX_SIZE + GT911_MAX_WRITE_SIZE];
    int32_t ret;

    if (num_bytes > GT911_MAX_WRITE_SIZE) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    tx_data[0] = (reg_addr >> 8) & 0xFF;
    tx_data[1] = (reg_addr) & 0xFF;
    if (num_bytes) {
        memcpy(&tx_data[GT911_REG_INDX_SIZE], reg_data, num_bytes);
    }

    /* Drop a completion left over from an aborted transfer */
    (void) xSemaphoreTake(gt911_drv_info.i2c_done, 0);

    ret = I2C_Driver->MasterTransmit(GT911_SLAVE_ADDR, tx_data, num_bytes + GT911_REG_INDX_SIZE,
                                     false);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return TOUCH_WaitI2C();
}

/**
  \fn           int32_t TOUCH_Read(uint16_t reg_addr, uint8_t *reg_data, uint32_t num_bytes)
  \brief        read register values from GT911 Touch screen registers.
                Uses the I2C write-read mode: the register address is sent from
                reg_data[0..1], then a repeated start reads num_bytes into reg_data.
  \param[in]    reg_addr : Register address.
  \param[out]   reg_data : buffer for the data, also holds the register address.
  \param[in]    num_bytes : Number of bytes to read, at least GT911_REG_INDX_SIZE.
  \return       \ref execution_status.
  */
static int32_t TOUCH_Read(uint16_t reg_addr, uint8_t *reg_data, uint32_t num_bytes)
{
    int32_t ret;

    if (num_bytes < GT911_REG_INDX_SIZE) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    reg_data[0] = (reg_addr >> 8) & 0xFF;
    reg_data[1] = (reg_addr) & 0xFF;

    /* Drop a completion left over from an aborted transfer */
    (void) xSemaphoreTake(gt911_drv_info.i2c_done, 0);

    ret = I2C_Driver->MasterReceive(GT911_SLAVE_ADDR, reg_data, num_bytes, false);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return TOUCH_WaitI2C();
}

/**
  \fn           int32_t TOUCH_IntEnable(bool enable)
  \brief        GT911 Touch screen INT line GPIO control.
  \param[in]    enable : true or false.
  \return       \ref execution_status.
  */
static int32_t TOUCH_IntEnable(bool enable)
{
    uint32_t arg = gt911_drv_info.int_irq_arg;
    int32_t  ret;

    if (enable) {
        ret = GPIO_Driver_INT->Control(BOARD_TOUCH_INT_GPIO_PIN, ARM_GPIO_ENABLE_INTERRUPT, &arg);
    } else {
        ret = GPIO_Driver_INT->Control(BOARD_TOUCH_INT_GPIO_PIN, ARM_GPIO_DISABLE_INTERRUPT, NULL);
    }

    return ret;
}

/**
  \fn           void TOUCH_GPIO_CB(uint32_t event)
  \brief        GT911 Touch screen GPIO callback event.
                The interrupt stays off until GetState has read the report.
  \param[in]    event: \ref GPIO Interrupt events.
  */
static void TOUCH_GPIO_CB(uint32_t event)
{
    ARG_UNUSED(event);
    (void) TOUCH_IntEnable(false);
    touch_event_gpio = true;
}

/**
  \fn           int32_t TOUCH_ArmAndClear(bool clear)
  \brief        Re-enable the INT interrupt, then clear the status register.
                Enabling first means the INT pulse of the next report cannot
                fall into a window where the interrupt is still off.
  \param[in]    clear : clear the status register after enabling the interrupt.
  \return       \ref execution_status.
  */
static int32_t TOUCH_ArmAndClear(bool clear)
{
    uint8_t data = 0;
    int32_t ret;

    ret = TOUCH_IntEnable(true);
    gt911_drv_info.armed_tick = xTaskGetTickCount();

    if (clear) {
        /*Clear status register*/
        int32_t clear_ret = TOUCH_Write(GT911_TOUCH_STATUS, &data, 1);
        if (ret == ARM_DRIVER_OK) {
            ret = clear_ret;
        }
    }

    return ret;
}

/**
  \fn           int32_t TOUCH_Reset(void)
  \brief        Hard Reset GT911 Touch Screen and latch its I2C address.
                The INT level at the rising edge of RST selects the address
                (low: 0x5D, high: 0x14); INT is then held low for the INT sync.
  \param[in]    none.
  \return       \ref execution_status.
  */
static int32_t TOUCH_Reset(void)
{
    int32_t ret;

    /* TOUCH INT pin: driven during reset, so its interrupt must be off */
    ret = GPIO_Driver_INT->Control(BOARD_TOUCH_INT_GPIO_PIN, ARM_GPIO_DISABLE_INTERRUPT, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_INT->SetValue(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_INT->SetDirection(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* TOUCH RST pin */
    ret = GPIO_Driver_RST->SetValue(BOARD_TOUCH_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_RST->SetDirection(BOARD_TOUCH_RESET_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    TOUCH_DelayMs(GT911_RESET_LOW_MS);

#if (GT911_SLAVE_ADDR == 0x14)
    ret = GPIO_Driver_INT->SetValue(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }
#endif

    sys_busy_loop_us(GT911_ADDR_SETUP_US);

    ret = GPIO_Driver_RST->SetValue(BOARD_TOUCH_RESET_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_HIGH);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    TOUCH_DelayMs(GT911_RESET_HIGH_MS);

    /* INT sync */
    ret = GPIO_Driver_INT->SetValue(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    TOUCH_DelayMs(GT911_INT_SYNC_MS);

    ret = GPIO_Driver_INT->SetDirection(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_DIRECTION_INPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t TOUCH_ReadInfo(void)
  \brief        Check the product id, read the panel configuration and log it.
                Selects the INT interrupt mode and release timeout from the config.
  \param[in]    none.
  \return       \ref execution_status.
  */
static int32_t TOUCH_ReadInfo(void)
{
    static const char *const trigger_name[] = {"rising", "falling", "low", "high"};
    uint8_t  id[6];
    uint8_t  cfg[GT911_CFG_READ_SIZE];
    uint16_t fw_version;
    uint16_t x_max;
    uint16_t y_max;
    uint8_t  trigger;
    uint32_t report_ms;
    int32_t  ret;

    ret = TOUCH_Read(GT911_PRODUCT_ID, id, sizeof(id));
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* "911" product id is in ascii character*/
    if (memcmp(id, "911", 3) != 0) {
        return ARM_DRIVER_ERROR;
    }

    fw_version = (uint16_t) (id[4] | (id[5] << 8));

    ret = TOUCH_Read(GT911_CONFIG_REG, cfg, sizeof(cfg));
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    x_max     = (uint16_t) (cfg[GT911_CFG_X_MAX] | (cfg[GT911_CFG_X_MAX + 1] << 8));
    y_max     = (uint16_t) (cfg[GT911_CFG_Y_MAX] | (cfg[GT911_CFG_Y_MAX + 1] << 8));
    trigger   = cfg[GT911_CFG_MODULE_SWITCH1] & GT911_INT_TRIGGER_MSK;
    report_ms = 5U + (cfg[GT911_CFG_REFRESH_RATE] & 0x0FU);

    switch (trigger) {
    case GT911_INT_TRIGGER_RISING:
        gt911_drv_info.int_irq_arg = ARM_GPIO_IRQ_POLARITY_HIGH |
                                     ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE |
                                     ARM_GPIO_IRQ_SENSITIVE_EDGE;
        break;
    case GT911_INT_TRIGGER_FALLING:
        gt911_drv_info.int_irq_arg = ARM_GPIO_IRQ_POLARITY_LOW |
                                     ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE |
                                     ARM_GPIO_IRQ_SENSITIVE_EDGE;
        break;
    case GT911_INT_TRIGGER_LOW:
        gt911_drv_info.int_irq_arg = ARM_GPIO_IRQ_POLARITY_LOW | ARM_GPIO_IRQ_SENSITIVE_LEVEL;
        break;
    default:
        gt911_drv_info.int_irq_arg = ARM_GPIO_IRQ_POLARITY_HIGH | ARM_GPIO_IRQ_SENSITIVE_LEVEL;
        break;
    }

    gt911_drv_info.release_timeout = pdMS_TO_TICKS(GT911_RELEASE_PERIODS * report_ms) + 1U;

    printf("GT911: fw 0x%04X, config v%u, %ux%u, INT %s, report %lu ms\n", fw_version,
           cfg[GT911_CFG_VERSION], x_max, y_max, trigger_name[trigger],
           (unsigned long) report_ms);

#if defined(RTE_PANEL_HACTIVE_TIME) && defined(RTE_PANEL_VACTIVE_LINE)
    if ((x_max != RTE_PANEL_HACTIVE_TIME) || (y_max != RTE_PANEL_VACTIVE_LINE)) {
        printf("GT911: panel reports %ux%u but the display is %ux%u, "
               "touch coordinates may need a transform\n",
               x_max, y_max, RTE_PANEL_HACTIVE_TIME, RTE_PANEL_VACTIVE_LINE);
    }
#endif

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t TOUCH_Init(void)
  \brief        Initialize GT911 Touch Screen.
                This function will
                    - initialize i2c instance.
                    - initialize gpio interrupt and reset instance.
  \param[in]    none.
  \return       \ref execution_status.
  */
static int32_t TOUCH_Init(void)
{
    int32_t ret;

    if (gt911_drv_info.i2c_done == NULL) {
        gt911_drv_info.i2c_done = xSemaphoreCreateBinary();
        if (gt911_drv_info.i2c_done == NULL) {
            return ARM_DRIVER_ERROR;
        }
    }

    ret = I2C_Driver->Initialize(TOUCH_I2C_CB);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = I2C_Driver->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* PowerControl applies RTE_I2Cx_IRQ_PRIORITY (0), which is above the FreeRTOS
     * syscall ceiling. The callback uses xSemaphoreGiveFromISR(), so the IRQ
     * must sit at or below configMAX_SYSCALL_INTERRUPT_PRIORITY. */
    NVIC_SetPriority(GT911_I2C_IRQN(BOARD_GT911_TOUCHSCREEN_I2C_INSTANCE),
                     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    ret = I2C_Driver->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Every MasterReceive writes the 2-byte register address first, then reads
     * after a repeated start, in one transfer */
    ret = I2C_Driver->Control(ARM_I2C_MODE_WRITE_READ,
                              ARM_I2C_WRITE_READ_MODE_EN | GT911_REG_INDX_SIZE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_INT->Initialize(BOARD_TOUCH_INT_GPIO_PIN, &TOUCH_GPIO_CB);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_INT->PowerControl(BOARD_TOUCH_INT_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_RST->Initialize(BOARD_TOUCH_RESET_GPIO_PIN, NULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_RST->PowerControl(BOARD_TOUCH_RESET_GPIO_PIN, ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn           int32_t TOUCH_Setup(void)
  \brief        Setup GT911 Touch Screen.
                This function will
  -                reset the GT911 and check its product id.
  -                read the panel configuration.
  -                discard any pending report and enable the INT interrupt.
  \param[in]    none.
  \return       \ref execution_status.
  */
static int32_t TOUCH_Setup(void)
{
    uint8_t data = GT911_CMD_READ_COORD;
    int32_t ret;

    ret = TOUCH_Reset();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = TOUCH_ReadInfo();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = TOUCH_Write(GT911_COMMAND_REG, &data, 1);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    gt911_drv_info.last.numtouches = 0;
    gt911_drv_info.poll_again      = false;
    touch_event_gpio               = false;

    /* Discard any report made before the interrupt was enabled */
    return TOUCH_ArmAndClear(true);
}

/**
  \fn           int32_t TOUCH_Sleep(void)
  \brief        Put the GT911 into sleep mode. It is woken by the next TOUCH_Setup reset.
  \param[in]    none.
  \return       \ref execution_status.
  */
static int32_t TOUCH_Sleep(void)
{
    uint8_t data = GT911_CMD_SCREEN_OFF;
    int32_t ret;

    /* INT must be driven low before the sleep command */
    ret = GPIO_Driver_INT->SetValue(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_OUTPUT_STATE_LOW);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = GPIO_Driver_INT->SetDirection(BOARD_TOUCH_INT_GPIO_PIN, GPIO_PIN_DIRECTION_OUTPUT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    TOUCH_DelayMs(GT911_SLEEP_INT_LOW_MS);

    return TOUCH_Write(GT911_COMMAND_REG, &data, 1);
}

/**
  \fn           void TOUCH_Deinit(void)
  \brief        Release the I2C and GPIO resources, ignoring errors so that a
                partial initialization is also undone.
  \param[in]    none.
  */
static void TOUCH_Deinit(void)
{
    (void) GPIO_Driver_INT->Control(BOARD_TOUCH_INT_GPIO_PIN, ARM_GPIO_DISABLE_INTERRUPT, NULL);
    (void) GPIO_Driver_INT->PowerControl(BOARD_TOUCH_INT_GPIO_PIN, ARM_POWER_OFF);
    (void) GPIO_Driver_INT->Uninitialize(BOARD_TOUCH_INT_GPIO_PIN);

    (void) GPIO_Driver_RST->PowerControl(BOARD_TOUCH_RESET_GPIO_PIN, ARM_POWER_OFF);
    (void) GPIO_Driver_RST->Uninitialize(BOARD_TOUCH_RESET_GPIO_PIN);

    (void) I2C_Driver->PowerControl(ARM_POWER_OFF);
    (void) I2C_Driver->Uninitialize();

    if (gt911_drv_info.i2c_done != NULL) {
        vSemaphoreDelete(gt911_drv_info.i2c_done);
        gt911_drv_info.i2c_done = NULL;
    }
}

/**
  \fn           int32_t TOUCH_ReadReport(bool lifted_if_idle, bool *ready)
  \brief        Read the status and touch points in as few transfers as possible
                and update the cached state. The caller clears a ready report.
  \param[in]    lifted_if_idle : treat "no report ready" as all fingers lifted.
  \param[out]   ready : the status register holds a report that must be cleared,
                also when reading its points failed.
  \return       \ref execution_status.
  */
static int32_t TOUCH_ReadReport(bool lifted_if_idle, bool *ready)
{
    uint8_t  buf[1 + (GT911_POINT_SIZE * RTE_ACTIVE_TOUCH_POINTS)];
    uint8_t  status;
    uint32_t count;
    int32_t  ret;

    *ready = false;

    /* Status and the first point in one read: covers the single-finger case */
    ret = TOUCH_Read(GT911_TOUCH_STATUS, buf, 1 + GT911_POINT_SIZE);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    status = buf[0];

    if (!(status & GT911_STATUS_BUF_READY)) {
        /* A held finger keeps producing reports, so silence means it was lifted */
        if (lifted_if_idle) {
            gt911_drv_info.last.numtouches = 0;
        }
        return ARM_DRIVER_OK;
    }

    *ready = true;
    count  = status & GT911_NUM_TOUCH_POINTS_MSK;

    /* A corrupt report is dropped (the caller still clears it) */
    if (count > GT911_MAX_TOUCH_POINTS) {
        return ARM_DRIVER_OK;
    }

    if (count > RTE_ACTIVE_TOUCH_POINTS) {
        count = RTE_ACTIVE_TOUCH_POINTS;
    }

    if (count > 1) {
        ret = TOUCH_Read(GT911_TOUCH_STATUS + 1 + GT911_POINT_SIZE, &buf[1 + GT911_POINT_SIZE],
                         (count - 1) * GT911_POINT_SIZE);
        if (ret != ARM_DRIVER_OK) {
            return ret;
        }
    }

    for (uint32_t i = 0; i < count; i++) {
        const uint8_t *point = &buf[1 + (i * GT911_POINT_SIZE)];

        gt911_drv_info.last.coordinates[i].x =
            (int16_t) (point[GT911_POINT_X] | (point[GT911_POINT_X + 1] << 8));
        gt911_drv_info.last.coordinates[i].y =
            (int16_t) (point[GT911_POINT_Y] | (point[GT911_POINT_Y + 1] << 8));
    }
    gt911_drv_info.last.numtouches = (int32_t) count;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t  ARM_TOUCH_SCREEN_Initialize (void)
  \brief       Initialize touch screen Interface.
  \param[in]   none.
  \return      \ref execution_status.
*/
static int32_t ARM_TOUCH_SCREEN_Initialize(void)
{
    int32_t ret;

    if (gt911_drv_info.touch_drv_status & GT911_FLAG_DRV_INIT_DONE) {
        return ARM_DRIVER_OK;
    }

    ret = TOUCH_Init();
    if (ret != ARM_DRIVER_OK) {
        TOUCH_Deinit();
        return ret;
    }

    gt911_drv_info.touch_drv_status = GT911_FLAG_DRV_INIT_DONE;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_TOUCH_SCREEN_PowerControl (ARM_POWER_STATE state)
  \brief       Control touch screen Interface Power.
  \param[in]   state  Power state.
  \return      \ref execution_status.
*/
static int32_t ARM_TOUCH_SCREEN_PowerControl(ARM_POWER_STATE state)
{
    int32_t ret;

    if (!(gt911_drv_info.touch_drv_status & GT911_FLAG_DRV_INIT_DONE)) {
        return ARM_DRIVER_ERROR;
    }

    switch (state) {
    case ARM_POWER_OFF:
        {
            /*Disabling the IRQs*/
            ret = TOUCH_IntEnable(false);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }

            if (!(gt911_drv_info.touch_drv_status & GT911_FLAG_POWER_ENABLED)) {
                return ARM_DRIVER_OK;
            }

            gt911_drv_info.touch_drv_status &= ~GT911_FLAG_POWER_ENABLED;
            gt911_drv_info.last.numtouches   = 0;
            touch_event_gpio                 = false;

            ret = TOUCH_Sleep();
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }

            break;
        }

    case ARM_POWER_FULL:
        {
            if (gt911_drv_info.touch_drv_status & GT911_FLAG_POWER_ENABLED) {
                return ARM_DRIVER_OK;
            }

            ret = TOUCH_Setup();
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }

            gt911_drv_info.touch_drv_status |= GT911_FLAG_POWER_ENABLED;

            break;
        }

    case ARM_POWER_LOW:
    default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_TOUCH_SCREEN_Uninitialize (void)
  \brief       uninitialize touch screen Interface.
  \return      \ref execution_status.
*/
static int32_t ARM_TOUCH_SCREEN_Uninitialize(void)
{
    if (!(gt911_drv_info.touch_drv_status & GT911_FLAG_DRV_INIT_DONE)) {
        return ARM_DRIVER_OK;
    }

    if (gt911_drv_info.touch_drv_status & GT911_FLAG_POWER_ENABLED) {
        (void) ARM_TOUCH_SCREEN_PowerControl(ARM_POWER_OFF);
    }

    TOUCH_Deinit();

    gt911_drv_info.touch_drv_status = 0;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_TOUCH_SCREEN_GetState (ARM_TOUCH_STATE *state)
  \brief       Get touch screen touch state.
               Returns the last report until the GT911 sends a new one, so
               "no new report" is not mistaken for "finger lifted".
  \param[out]  state pointer to ARM_TOUCH_STATE, filled also when an error is returned.
  \return      \ref execution_status.
  */
static int32_t ARM_TOUCH_SCREEN_GetState(ARM_TOUCH_STATE *state)
{
    int32_t ret = ARM_DRIVER_OK;

    if (state == NULL) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(gt911_drv_info.touch_drv_status & GT911_FLAG_POWER_ENABLED)) {
        return ARM_DRIVER_ERROR;
    }

    /* Pressed, but no INT for GT911_RELEASE_PERIODS report periods: the
     * release report (or its INT edge) was missed */
    bool int_event = touch_event_gpio;
    bool stale     = (gt911_drv_info.last.numtouches > 0) &&
                     ((xTaskGetTickCount() - gt911_drv_info.armed_tick) >=
                      gt911_drv_info.release_timeout);

    if (int_event || stale || gt911_drv_info.poll_again) {
        bool ready;

        touch_event_gpio = false;

        ret              = TOUCH_ReadReport(stale && !int_event, &ready);

        /* re-enable gt911 interrupt on every path: the ISR disabled it */
        int32_t arm_ret  = TOUCH_ArmAndClear(ready);
        if (ret == ARM_DRIVER_OK) {
            ret = arm_ret;
        }

        /* Retry after an I2C error, or once if the INT fired before the
         * report was ready */
        gt911_drv_info.poll_again = (ret != ARM_DRIVER_OK) || (int_event && !ready);
    }

    *state = gt911_drv_info.last;

    return ret;
}

extern ARM_DRIVER_TOUCH_SCREEN GT911;
ARM_DRIVER_TOUCH_SCREEN        GT911 = {
    ARM_TOUCH_SCREEN_GetVersion,
    ARM_TOUCH_SCREEN_GetCapabilities,
    ARM_TOUCH_SCREEN_Initialize,
    ARM_TOUCH_SCREEN_Uninitialize,
    ARM_TOUCH_SCREEN_PowerControl,
    ARM_TOUCH_SCREEN_GetState
};
