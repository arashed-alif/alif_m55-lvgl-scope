#include "drv_i3c_bus.h"

#include <stdbool.h>
#include <stdio.h>

#include "Driver_I3C.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Longest single transfer: 9 bytes at 400 kHz is ~250 us, so this only trips
 * on a hung bus or a missing callback. */
#define I3C_BUS_TIMEOUT_MS   20U
/* Longest wait for another task's transfer to finish. */
#define I3C_BUS_LOCK_MS      50U

extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

static SemaphoreHandle_t bus_mutex;
/* Task blocked in bus_wait(); the I3C ISR notifies it directly. Only one
 * transfer runs at a time (bus_mutex), so one waiter is enough. */
static TaskHandle_t bus_waiter;
static bool bus_ready;

static uint8_t bus_devices[I3C_BUS_MAX_DEVS];
static uint8_t bus_device_count;

/* Runs in the I3C ISR: hand the event bits straight to the waiting task. */
static void bus_cb(uint32_t event)
{
    BaseType_t woken = pdFALSE;

    if (bus_waiter != NULL) {
        (void)xTaskNotifyFromISR(bus_waiter, event, eSetBits, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

static bool bus_lock(void)
{
    if (!bus_ready || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return false;
    }
    return xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(I3C_BUS_LOCK_MS)) == pdTRUE;
}

static void bus_unlock(void)
{
    (void)xSemaphoreGive(bus_mutex);
}

/* Arm the notification for the calling task. Call before starting a transfer. */
static void bus_arm(void)
{
    bus_waiter = xTaskGetCurrentTaskHandle();
    (void)ulTaskNotifyValueClear(NULL, UINT32_MAX);
}

/* Block until the I3C ISR reports completion (or timeout). Returns -1 on
 * timeout or transfer error (NACK). */
static int32_t bus_wait(uint8_t addr)
{
    uint32_t flags = 0U;
    TickType_t timeout = pdMS_TO_TICKS(I3C_BUS_TIMEOUT_MS);

    if (timeout == 0U) {
        timeout = 1U;
    }

    if (xTaskNotifyWait(0U, UINT32_MAX, &flags, timeout) != pdTRUE) {
        bus_waiter = NULL;
        printf("Error: I3C transfer timeout after %ums, addr 0x%02X\n",
               (unsigned int)I3C_BUS_TIMEOUT_MS, addr);
        return -1;
    }
    bus_waiter = NULL;

    if ((flags & ARM_I3C_EVENT_TRANSFER_ERROR) != 0U) {
        return -1;
    }
    return 0;
}

/* One transfer with the bus already held. */
static int32_t bus_transmit(uint8_t addr, const uint8_t *tx, uint16_t len)
{
    bus_arm();
    if (I3Cdrv->MasterTransmit(addr, tx, len) != ARM_DRIVER_OK) {
        bus_waiter = NULL;
        printf("Error: I3C MasterTransmit failed, addr 0x%02X\n", addr);
        return -1;
    }
    return bus_wait(addr);
}

static int32_t bus_receive(uint8_t addr, uint8_t *rx, uint16_t len)
{
    bus_arm();
    if (I3Cdrv->MasterReceive(addr, rx, len) != ARM_DRIVER_OK) {
        bus_waiter = NULL;
        printf("Error: I3C MasterReceive failed, addr 0x%02X\n", addr);
        return -1;
    }
    return bus_wait(addr);
}

int32_t i3c_bus_init(void)
{
    int32_t ret;

    if (bus_ready) {
        return 0;
    }

    if (bus_mutex == NULL) {
        bus_mutex = xSemaphoreCreateMutex();
        if (bus_mutex == NULL) {
            printf("\r\n Error: I3C bus mutex create failed.\r\n");
            return -1;
        }
    }

    ret = I3Cdrv->Initialize(bus_cb);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Initialize failed.\r\n");
        return ret;
    }
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Power Up failed.\r\n");
        return ret;
    }
    /* PowerControl applies RTE_I3C_IRQ_PRI (0), which is above the FreeRTOS
     * syscall ceiling. The callback uses xTaskNotifyFromISR(), so the IRQ
     * must sit at or below configMAX_SYSCALL_INTERRUPT_PRIORITY. */
    NVIC_SetPriority(I3C_IRQ_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    ret = I3Cdrv->Control(I3C_MASTER_INIT, 0);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Control: Master Init failed.\r\n");
        return ret;
    }
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,
                          I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Control: Set Bus Mode failed.\r\n");
        return ret;
    }

    bus_device_count = 0U;
    bus_ready = true;
    return 0;
}

/* Attach/detach run during task start-up; the mutex keeps two tasks from
 * editing the device table at once. Before the scheduler runs (or if the lock
 * times out) they go ahead unlocked, which is safe while single threaded. */
int32_t i3c_bus_attach_i2c(uint8_t addr)
{
    bool locked = bus_lock();
    int32_t ret = 0;

    if (!bus_ready) {
        return -1;
    }

    for (uint8_t i = 0U; i < bus_device_count; i++) {
        if (bus_devices[i] == addr) {
            goto out;
        }
    }
    if (bus_device_count >= I3C_BUS_MAX_DEVS) {
        printf("\r\n Error: I3C device table full.\r\n");
        ret = -1;
        goto out;
    }

    ret = I3Cdrv->AttachSlvDev(ARM_I3C_DEVICE_TYPE_I2C, addr);
    if (ret != ARM_DRIVER_OK) {
        printf("\r\n Error: I3C Attach I2C device 0x%02X failed.\r\n", addr);
        goto out;
    }
    bus_devices[bus_device_count++] = addr;

out:
    if (locked) {
        bus_unlock();
    }
    return ret;
}

int32_t i3c_bus_detach(uint8_t addr)
{
    bool locked = bus_lock();
    int32_t ret = 0;

    for (uint8_t i = 0U; i < bus_device_count; i++) {
        if (bus_devices[i] == addr) {
            ret = I3Cdrv->Detachdev(addr);
            if (ret != ARM_DRIVER_OK) {
                printf("\r\n Error: I3C Detach I2C device 0x%02X failed.\r\n", addr);
                break;
            }
            bus_devices[i] = bus_devices[--bus_device_count];
            break;
        }
    }

    if (locked) {
        bus_unlock();
    }
    return ret;
}

int32_t i3c_bus_write(uint8_t addr, const uint8_t *tx, uint16_t len)
{
    int32_t ret;

    if (!bus_lock()) {
        return -1;
    }
    ret = bus_transmit(addr, tx, len);
    bus_unlock();
    return ret;
}

int32_t i3c_bus_read(uint8_t addr, uint8_t *rx, uint16_t len)
{
    int32_t ret;

    if (!bus_lock()) {
        return -1;
    }
    ret = bus_receive(addr, rx, len);
    bus_unlock();
    return ret;
}

int32_t i3c_bus_write_read(uint8_t addr, const uint8_t *tx, uint16_t tx_len,
                           uint8_t *rx, uint16_t rx_len)
{
    int32_t ret;

    if (!bus_lock()) {
        return -1;
    }
    ret = bus_transmit(addr, tx, tx_len);
    if (ret == 0) {
        ret = bus_receive(addr, rx, rx_len);
    }
    bus_unlock();
    return ret;
}
