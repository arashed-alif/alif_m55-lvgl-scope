#include "drv_ina228.h"
#include "Driver_I3C.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include "RTE_Components.h"
#include CMSIS_device_header

#define INA228_TX_COUNT     3
#define INA228_RX_COUNT     1
#define INA228_MAX_RX_COUNT 8
#define INA228_I2C_TIMEOUT  20

/* 1: print every successful transfer and the received bytes (slow: ~5 ms
 * per line at 115200 baud). Error prints are always enabled. */
#define INA228_VERBOSE      0

/* Task blocked in wait_for_i3c_event(); the I3C ISR notifies it directly. */
static TaskHandle_t i3c_waiter;

static uint32_t current_lsb = 5198;		/* nA per LSB */

/* CONFIG[4] ADCRANGE, tracked by INA228_WriteCONFIG so that the VSHUNT scaling
 * and the SHUNT_CAL value follow the selected full scale range automatically */
static uint8_t adc_range = 0;

/* VSHUNT resolution as exact fraction: 625/2 = 312.5 nV/LSB (ADCRANGE = 0),
 * 625/8 = 78.125 nV/LSB (ADCRANGE = 1) */
static uint32_t voltage_res_num = 625;
static uint32_t voltage_res_den = 2;

extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* Arm the notification for the calling task. Call before starting a transfer. */
static void arm_i3c_wait(void)
{
	i3c_waiter = xTaskGetCurrentTaskHandle();
	(void)ulTaskNotifyValueClear(NULL, UINT32_MAX);
}

/* Block until the I3C ISR reports completion (or timeout). Returns the
 * event flags via *events, -1 on timeout or transfer error. */
static int32_t wait_for_i3c_event(uint32_t *events)
{
	uint32_t flags = 0U;
	TickType_t timeout = pdMS_TO_TICKS(INA228_I2C_TIMEOUT);

	if (timeout == 0U) {
		timeout = 1U;
	}

	if (xTaskNotifyWait(0U, UINT32_MAX, &flags, timeout) != pdTRUE) {
		i3c_waiter = NULL;
		printf("Error: I3C transfer timeout after %dms\n",
			   INA228_I2C_TIMEOUT);
		return -1;
	}
	i3c_waiter = NULL;
	*events = flags;

	if ((flags & ARM_I3C_EVENT_TRANSFER_ERROR) != 0U) {
		return -1;
	}

	return 0;
}

static int32_t _INA228_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t num_bytes) {
	uint8_t  tx_data[INA228_TX_COUNT] = {0};
	uint8_t  static reg_addr_last_used = 0;
	uint32_t events = 0U;
	int32_t  ret;

	tx_data[0] = reg_addr & 0x7F;
	arm_i3c_wait();

	if (num_bytes) {
		tx_data[1] = reg_data[1];
		tx_data[2] = reg_data[0];
		ret = I3Cdrv->MasterTransmit(dev_addr, tx_data, INA228_TX_COUNT);
		if(ret != ARM_DRIVER_OK)
		{
			printf("\r\n Error: i2c Master Transmit failed. \r\n");
			return ret;
		}
	} else {
		if (reg_addr_last_used == reg_addr) return 0;
		ret = I3Cdrv->MasterTransmit(dev_addr, tx_data, INA228_RX_COUNT);
		if(ret != ARM_DRIVER_OK)
		{
			printf("\r\n Error: i2c Master Receive failed. \r\n");
			return ret;
		}
	}

	if (wait_for_i3c_event(&events) != 0) {
		/* TX Error: Got NACK from slave, or timeout */
		printf(">> I2C Master Transmit Error: slave addr: 0x%X\n", dev_addr);
		return -1;
	}

#if INA228_VERBOSE
	if (events & ARM_I3C_EVENT_TRANSFER_DONE) {
		/* TX Success: Got ACK from slave */
		printf(">> I2C Master Transmit Success: Got ACK from slave addr: 0x%X\n", dev_addr);
	}
#else
	(void)events;
#endif

	reg_addr_last_used = reg_addr;
	return 0;
}

static int32_t _INA228_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t num_bytes) {
	uint8_t  rx_data[INA228_MAX_RX_COUNT] = {0};
	uint32_t events = 0U;
	int32_t  ret;

	if (_INA228_Write(dev_addr, reg_addr, 0, 0)) return -1;

	arm_i3c_wait();
	ret = I3Cdrv->MasterReceive(dev_addr, &rx_data[num_bytes-2], num_bytes);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: i2c Master Receive failed. \r\n");
		i3c_waiter = NULL;
		return -1;
	}

	if (wait_for_i3c_event(&events) != 0) {
		printf(">> I2C Master Receive Error: slave addr: 0x%X\n", dev_addr);
		return -1;
	}

	switch(num_bytes) {
	case INA228_16BREG:
		num_bytes = 2;
		reg_data[0] = rx_data[1];
		reg_data[1] = rx_data[0];
		break;
	case INA228_24BREG:
		num_bytes = 4;
		reg_data[0] = rx_data[3];
		reg_data[1] = rx_data[2];
		reg_data[2] = rx_data[1];
		reg_data[3] = rx_data[0];
		break;
	case INA228_40BREG:
		num_bytes = 8;
		reg_data[0] = rx_data[7];
		reg_data[1] = rx_data[6];
		reg_data[2] = rx_data[5];
		reg_data[3] = rx_data[4];
		reg_data[4] = rx_data[3];
		reg_data[5] = rx_data[2];
		reg_data[6] = rx_data[1];
		reg_data[7] = rx_data[0];
		break;
	default:
		printf("\r\n Error: INA228 unsupported register width %u\r\n", num_bytes);
		return -1;
	}

#if INA228_VERBOSE
	if (events & ARM_I3C_EVENT_TRANSFER_DONE) {
		uint8_t i;
		/* RX Success: Got ACK from slave */
		printf(">> I2C Master Receive Success: Got ACK from slave addr: 0x%X\n", dev_addr);
		printf(">> I2C Master Received Data from slave: 0x");
		for (i = 0; i < num_bytes; i++) {
			printf("%02X", rx_data[i]);
		}
		printf("\n");
	}
#else
	(void)events;
#endif

	return 0;
}

/* Runs in the I3C ISR: hand the event bits straight to the waiting task. */
static void INA228_CB(uint32_t event)
{
	BaseType_t woken = pdFALSE;

	if (i3c_waiter != NULL) {
		(void)xTaskNotifyFromISR(i3c_waiter, event, eSetBits, &woken);
	}
	portYIELD_FROM_ISR(woken);
}

int32_t INA228_Init(uint8_t *dev_addr, uint8_t dev_count)
{
    int32_t ret;
    ret = I3Cdrv->Initialize(INA228_CB);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Initialize failed.\r\n");
		return ret;
    }
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Power Up failed.\r\n");
		return ret;
    }
    /* PowerControl applies RTE_I3C_IRQ_PRI (0), which is above the FreeRTOS
     * syscall ceiling. The callback uses xTaskNotifyFromISR(), so the IRQ
     * must sit at or below configMAX_SYSCALL_INTERRUPT_PRIORITY. */
    NVIC_SetPriority(I3C_IRQ_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    ret = I3Cdrv->Control(I3C_MASTER_INIT, 0);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Control: Master Init failed.\r\n");
		return ret;
    }
    ret = I3Cdrv->Control(I3C_MASTER_SET_BUS_MODE,
                           I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Control: Set Bus Mode failed.\r\n");
		return ret;
    }

	while(dev_count--) {
        ret = I3Cdrv->AttachSlvDev(ARM_I3C_DEVICE_TYPE_I2C, *(dev_addr++));
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Attach I2C device failed.\r\n");
			return ret;
        }
	}
	return 0;
}

int32_t INA228_Deinit(uint8_t *dev_addr, uint8_t dev_count)
{
	int32_t  ret;
	while(dev_count--) {
		ret = I3Cdrv->Detachdev(*(dev_addr++));
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Detach I2C device failed.\r\n");
			return ret;
        }
	}

    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Power OFF failed.\r\n");
		return ret;
    }

    ret = I3Cdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Uninitialize failed.\r\n");
		return ret;
    }

	return 0;
}

int32_t INA228_Write(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data) {
	return _INA228_Write(dev_addr, reg_addr, (uint8_t *) &reg_data, INA228_16BREG);
}

int32_t INA228_Read16(uint8_t dev_addr, uint8_t reg_addr, uint16_t *reg_data) {
	return _INA228_Read(dev_addr, reg_addr, (uint8_t *) reg_data, INA228_16BREG);
}

int32_t INA228_Read32(uint8_t dev_addr, uint8_t reg_addr, uint32_t *reg_data) {
	return _INA228_Read(dev_addr, reg_addr, (uint8_t *) reg_data, INA228_24BREG);
}

int32_t INA228_Read64(uint8_t dev_addr, uint8_t reg_addr, uint64_t *reg_data) {
	return _INA228_Read(dev_addr, reg_addr, (uint8_t *) reg_data, INA228_40BREG);
}

int32_t INA228_WriteCONFIG(uint8_t dev_addr, uint16_t *reg_data) {
	adc_range = (*reg_data >> 4) & 1U;
	voltage_res_den = adc_range ? 8 : 2;	/* 78.125 nV/LSB : 312.5 nV/LSB */
	return INA228_Write(dev_addr, CONFIG, *reg_data);
}

/* takes SHUNT_CAL computed for ADCRANGE = 0 and applies the x4 scaling the
 * device requires when ADCRANGE = 1; the register is 15 bits wide, and CONFIG
 * must have been written first so that adc_range is current */
int32_t INA228_WriteSHUNT_CAL(uint8_t dev_addr, uint16_t shunt_cal) {
	if (adc_range) shunt_cal <<= 2;
	return INA228_Write(dev_addr, SHUNT_CAL, shunt_cal & 0x7FFF);
}

/* returns measured VSHUNT as nano-volts */
int32_t INA228_ReadVSHUNT(uint8_t dev_addr, int32_t *reg_data) {
	uint32_t raw = 0;
	int32_t ret = INA228_Read32(dev_addr, VSHUNT, &raw);
	int32_t val = (int32_t)(raw << 8) >> 12;	/* sign-extend 20-bit two's complement at bits [23:4] */
	*reg_data = (int32_t)(((int64_t)val * voltage_res_num) / voltage_res_den);
	return ret;
}

/* returns measured VBUS as micro-volts */
int32_t INA228_ReadVBUS(uint8_t dev_addr, uint32_t *reg_data) {
	int32_t ret = INA228_Read32(dev_addr, VBUS, reg_data);
	*reg_data >>= 4;
	*reg_data = (uint32_t)(((uint64_t)*reg_data * 3125) / 16);	/* 195.3125 = 3125/16 uV/LSB */
	return ret;
}

/* returns measured DIETEMP as millidegrees-C */
int32_t INA228_ReadDIETEMP(uint8_t dev_addr, int32_t *reg_data) {
	uint16_t raw;
	int32_t ret = INA228_Read16(dev_addr, DIETEMP, &raw);
	*reg_data = ((int32_t)(int16_t)raw * 125) / 16;	/* signed 16-bit reg; 7.8125 = 125/16 millidegrees-C/LSB */
	return ret;
}

/* returns measured CURRENT as nano-amps */
int32_t INA228_ReadCURRENT(uint8_t dev_addr, int32_t *reg_data) {
	uint32_t raw = 0;
	int32_t ret = INA228_Read32(dev_addr, CURRENT, &raw);
	int32_t val = (int32_t)(raw << 8) >> 12;	/* sign-extend 20-bit two's complement at bits [23:4] */
	*reg_data = val * (int32_t)current_lsb;
	return ret;
}

/* returns measured POWER as nano-Watts */
int32_t INA228_ReadPOWER(uint8_t dev_addr, uint64_t *reg_data) {
	uint32_t raw;
	int32_t ret = INA228_Read32(dev_addr, POWER, &raw);
	*reg_data = ((uint64_t)raw * current_lsb * 16) / 5;	/* 3.2 = 16/5 */
	return ret;
}

int32_t INA228_ClearAccumulators(uint8_t dev_addr) {
	uint16_t reg_data;
	INA228_Read16(dev_addr, CONFIG, &reg_data);
	reg_data |= 1U << 14;
	return INA228_Write(dev_addr, CONFIG, reg_data);
}

/* returns measured ENERGY as nano-Joules */
int32_t INA228_ReadENERGY(uint8_t dev_addr, uint64_t *reg_data) {
	int32_t ret = INA228_Read64(dev_addr, ENERGY, reg_data);
	*reg_data = (*reg_data * current_lsb * 256) / 5;	/* 3.2 * 16 = 51.2 = 256/5 */
	return ret;
}

/* returns measured CHARGE as nano-Coulombs */
int32_t INA228_ReadCHARGE(uint8_t dev_addr, int64_t *reg_data) {
	uint64_t raw = 0;
	int32_t ret = INA228_Read64(dev_addr, CHARGE, &raw);
	int64_t val = (int64_t)(raw << 24) >> 24;	/* sign-extend 40-bit two's complement */
	*reg_data = val * (int64_t)current_lsb;
	return ret;
}