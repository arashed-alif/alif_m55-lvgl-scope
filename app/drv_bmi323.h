/* Copyright (c) 2022 ALIF SEMICONDUCTOR

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#ifndef DRV_BMI323_H
#define DRV_BMI323_H

#include <stdint.h>

#define i2c_addr_68                 0b1101000
#define i2c_addr_69                 0b1101001

/* SDO is pulled high on the Alif kits: 0x69 (verified with the chip ID test). */
#define BMI323_I2C_ADDR             i2c_addr_69
#define BMI323_CHIP_ID              0x43

#define BMI323_STATUS_ACCRDY        0x80
#define BMI323_STATUS_GYRRDY        0x40
#define BMI323_STATUS_TMPRDY        0x20
#define BMI323_CMD_RESET            0xDEAF

/* INT_STATUS_INT1 / INT_STATUS_INT2 bits */
#define BMI323_INT_NO_MOTION        (1U << 0)
#define BMI323_INT_ANY_MOTION       (1U << 1)

typedef enum {
BMI323_CHIP_ID_REG,
BMI323_ERROR_REG,
BMI323_STATUS_REG,

BMI323_ACC_X_REG,
BMI323_ACC_Y_REG,
BMI323_ACC_Z_REG,

BMI323_GYR_X_REG,
BMI323_GYR_Y_REG,
BMI323_GYR_Z_REG,

BMI323_TEMP_DATA_REG,
BMI323_TIME_0_REG,
BMI323_TIME_1_REG,
BMI323_SAT_FLAGS,
BMI323_INT_STATUS_INT1,
BMI323_INT_STATUS_INT2,
BMI323_INT_STATUS_IBI,
BMI323_FEATURE_IO0,
BMI323_FEATURE_IO1,
BMI323_FEATURE_IO2,
BMI323_FEATURE_IO3,
BMI323_FEATURE_IO_STATUS,
BMI323_FIFO_FILL_LEVEL,
BMI323_FIFO_DATA,

BMI323_ACC_CONF = 0x20,
BMI323_GYR_CONF,

BMI323_ACC_CONF_ALT = 0x28,
BMI323_GYR_CONF_ALT,
BMI323_ALT_CONF,
BMI323_ALT_STATUS,
BMI323_FIFO_WATERMARK = 0x35,
BMI323_FIFO_CONF,
BMI323_FIFO_CTRL,
BMI323_IO_INT_CTRL,

BMI323_INT_CONF,
BMI323_INT_MAP1,
BMI323_INT_MAP2,

BMI323_FEATURE_CTRL = 0x40,
BMI323_FEATURE_DATA_ADDR,
BMI323_FEATURE_DATA_TX,

BMI323_CMD_REG = 0x7E,
} BMI323_REG_MAP_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} xyz_accel_s;

/* Any-motion / no-motion detector settings. The feature engine evaluates
 * 50 Hz data: thresholds are in 1/512 g, durations in 20 ms steps. */
typedef struct {
	uint16_t any_threshold;     /* 12 bits */
	uint16_t any_hysteresis;    /* 10 bits, below the threshold */
	uint16_t any_duration;      /* 13 bits: samples above threshold before firing */
	uint16_t no_threshold;
	uint16_t no_hysteresis;
	uint16_t no_duration;       /* samples below threshold before firing */
} bmi323_motion_config_t;

/* ISR callback for the INT1 line; runs in interrupt context. */
typedef void (*bmi323_irq_cb_t)(void);

/* All calls go through drv_i3c_bus, so they need a running scheduler.
 * Return 0 on success. */

/* Attach, check the chip ID, soft reset and start the feature engine. */
int32_t BMI323_Init(uint8_t dev_addr);
/* Program the motion detectors, run the accelerometer in low-power mode at
 * 50 Hz, and drive INT1 open-drain, active low, latched until
 * BMI323_ReadIntStatus1() reads it. Nothing is mapped to INT1 yet. */
int32_t BMI323_ConfigMotion(uint8_t dev_addr, const bmi323_motion_config_t *cfg);
/* Route BMI323_INT_NO_MOTION and/or BMI323_INT_ANY_MOTION to INT1. */
int32_t BMI323_MapInt1(uint8_t dev_addr, uint16_t int_mask);
/* Read (and so clear) INT_STATUS_INT1. */
int32_t BMI323_ReadIntStatus1(uint8_t dev_addr, uint16_t *status);
/* Raw accelerometer counts (8 g range: 4096 LSB/g). */
int32_t BMI323_ReadAcc(uint8_t dev_addr, xyz_accel_s *acc);

/* Raw 16-bit register access. */
int32_t BMI323_Read(uint8_t dev_addr, uint8_t reg_addr, uint16_t *reg_data, uint8_t num_words);
int32_t BMI323_Write(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data);

/* Configure the board's BMI323 IRQ GPIO (board_defs.h) for a falling edge on
 * INT1 and call cb from its interrupt. */
int32_t BMI323_IrqInit(bmi323_irq_cb_t cb);

#endif
