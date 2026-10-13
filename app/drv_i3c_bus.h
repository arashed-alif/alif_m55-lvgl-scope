#ifndef DRV_I3C_BUS_H
#define DRV_I3C_BUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Shared owner of the I3C controller (Driver_I3C). Every device driver on the
 * bus goes through here so that only one callback is ever installed and only
 * one transfer is in flight at a time.
 *
 * i3c_bus_init() must run once before any other call; main() does it before
 * the scheduler starts. All transfer calls block on a FreeRTOS mutex and on
 * the calling task's notification value, so they must only be called from a
 * task with the scheduler running, and that task must not use direct-to-task
 * notifications for anything else (use a semaphore instead). */

/* Bring the controller up as an I2C (mixed fast mode, 400 kHz) master.
 * Idempotent. Returns 0 on success. */
int32_t i3c_bus_init(void);

/* Attach / detach an I2C device. Attaching an address twice is a no-op. */
int32_t i3c_bus_attach_i2c(uint8_t addr);
int32_t i3c_bus_detach(uint8_t addr);

/* Single transfers. Return 0 on success, -1 on NACK, timeout or driver error. */
int32_t i3c_bus_write(uint8_t addr, const uint8_t *tx, uint16_t len);
int32_t i3c_bus_read(uint8_t addr, uint8_t *rx, uint16_t len);

/* Write then read with the bus held across both, so no other task's transfer
 * can land between a register address write and the read that follows it. */
int32_t i3c_bus_write_read(uint8_t addr, const uint8_t *tx, uint16_t tx_len,
                           uint8_t *rx, uint16_t rx_len);

#ifdef __cplusplus
}
#endif

#endif
