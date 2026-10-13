#include <stdio.h>
#include "drv_ina228.h"
#include "drv_i3c_bus.h"

#define INA228_TX_COUNT     3
#define INA228_RX_COUNT     1
#define INA228_MAX_RX_COUNT 8

/* 1: print every successful transfer and the received bytes (slow: ~5 ms
 * per line at 115200 baud). Error prints are always enabled. */
#define INA228_VERBOSE      0

/* CURRENT_LSB a device starts with until INA228_SetCurrentLSB() */
#define INA228_DEFAULT_CURRENT_LSB  5198U   /* nA per LSB */

/* 0xFF (no such register): the register pointer is unknown */
#define INA228_REG_UNKNOWN  0xFFU

/* Per-device state, so several INA228s with different shunts and ranges can
 * share the bus. Every public call looks its device up by I2C address and
 * fails for an address INA228_Init() has not registered. */
typedef struct {
    uint8_t  addr;
    /* CONFIG[4] ADCRANGE, tracked by INA228_WriteCONFIG so that the VSHUNT
     * scaling and the SHUNT_CAL value follow the selected full scale range
     * automatically */
    uint8_t  adc_range;
    /* Register pointer the INA228 holds, so a read of the same register can
     * skip the pointer write. INA228_REG_UNKNOWN at start-up (a warm reset
     * leaves the chip's pointer wherever it was) and after an error. */
    uint8_t  reg_addr_last_used;
    uint32_t current_lsb;       /* nA per LSB */
    /* VSHUNT resolution as exact fraction: 625/2 = 312.5 nV/LSB (ADCRANGE = 0),
     * 625/8 = 78.125 nV/LSB (ADCRANGE = 1) */
    uint32_t voltage_res_den;
} ina228_dev_t;

static const uint32_t voltage_res_num = 625;

static ina228_dev_t ina228_devs[INA228_MAX_DEVICES];
static uint8_t ina228_dev_count = 0;

static ina228_dev_t *ina228_find(uint8_t dev_addr) {
    for (uint8_t i = 0; i < ina228_dev_count; i++) {
        if (ina228_devs[i].addr == dev_addr) {
            return &ina228_devs[i];
        }
    }
    printf(">> INA228 Error: slave addr 0x%X not initialized\n", dev_addr);
    return NULL;
}

static int32_t _INA228_Write(ina228_dev_t *dev, uint8_t reg_addr, uint8_t *reg_data, uint8_t num_bytes) {
    uint8_t  tx_data[INA228_TX_COUNT] = {0};
    uint16_t tx_count = INA228_RX_COUNT;

    tx_data[0] = reg_addr & 0x7F;

    if (num_bytes) {
        tx_data[1] = reg_data[1];
        tx_data[2] = reg_data[0];
        tx_count = INA228_TX_COUNT;
    } else if (dev->reg_addr_last_used == reg_addr) {
        return 0;
    }

    if (i3c_bus_write(dev->addr, tx_data, tx_count) != 0) {
        /* TX Error: Got NACK from slave, or timeout */
        printf(">> I2C Master Transmit Error: slave addr: 0x%X\n", dev->addr);
        dev->reg_addr_last_used = INA228_REG_UNKNOWN;
        return -1;
    }

#if INA228_VERBOSE
    printf(">> I2C Master Transmit Success: Got ACK from slave addr: 0x%X\n", dev->addr);
#endif

    dev->reg_addr_last_used = reg_addr;
    return 0;
}

static int32_t _INA228_Read(ina228_dev_t *dev, uint8_t reg_addr, uint8_t *reg_data, uint8_t num_bytes) {
    uint8_t  rx_data[INA228_MAX_RX_COUNT] = {0};

    /* Only this driver talks to the INA228s, so a chip's register pointer
     * can't move between the pointer write and the read even though the bus
     * is released in between. The pointer is tracked per chip. */
    if (_INA228_Write(dev, reg_addr, 0, 0)) return -1;

    if (i3c_bus_read(dev->addr, &rx_data[num_bytes-2], num_bytes) != 0) {
        printf(">> I2C Master Receive Error: slave addr: 0x%X\n", dev->addr);
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
    {
        uint8_t i;
        /* RX Success: Got ACK from slave */
        printf(">> I2C Master Receive Success: Got ACK from slave addr: 0x%X\n", dev->addr);
        printf(">> I2C Master Received Data from slave: 0x");
        for (i = 0; i < num_bytes; i++) {
            printf("%02X", rx_data[i]);
        }
        printf("\n");
    }
#endif

    return 0;
}

/* The I3C controller itself is brought up by i3c_bus_init() (main.c); this
 * only registers the INA228 addresses with it and with this driver. It can be
 * called more than once: an address that is already registered just has its
 * register pointer forgotten. A new device starts with ADCRANGE = 0 and
 * INA228_DEFAULT_CURRENT_LSB. */
int32_t INA228_Init(uint8_t *dev_addr, uint8_t dev_count)
{
    int32_t ret = i3c_bus_init();

    if (ret != 0) {
        return ret;
    }
    while(dev_count--) {
        uint8_t addr = *(dev_addr++);
        ina228_dev_t *dev = NULL;

        for (uint8_t i = 0; i < ina228_dev_count; i++) {
            if (ina228_devs[i].addr == addr) {
                dev = &ina228_devs[i];
                break;
            }
        }
        if (dev == NULL) {
            if (ina228_dev_count == INA228_MAX_DEVICES) {
                printf(">> INA228 Error: slave addr 0x%X: more than %d devices\n",
                       addr, INA228_MAX_DEVICES);
                return -1;
            }
            ret = i3c_bus_attach_i2c(addr);
            if (ret != 0) {
                return ret;
            }
            dev = &ina228_devs[ina228_dev_count++];
            dev->addr            = addr;
            dev->adc_range       = 0;
            dev->current_lsb     = INA228_DEFAULT_CURRENT_LSB;
            dev->voltage_res_den = 2;
        }
        dev->reg_addr_last_used = INA228_REG_UNKNOWN;
    }
    return 0;
}

/* Detaches the INA228s only: the bus stays up for the other devices on it. */
int32_t INA228_Deinit(uint8_t *dev_addr, uint8_t dev_count)
{
    int32_t ret;
    while(dev_count--) {
        uint8_t addr = *(dev_addr++);

        ret = i3c_bus_detach(addr);
        if (ret != 0) {
            return ret;
        }
        for (uint8_t i = 0; i < ina228_dev_count; i++) {
            if (ina228_devs[i].addr == addr) {
                ina228_devs[i] = ina228_devs[--ina228_dev_count];
                break;
            }
        }
    }
    return 0;
}

/* CURRENT_LSB in nA, the value SHUNT_CAL was computed for. Scales CURRENT,
 * POWER, ENERGY and CHARGE of this device. */
int32_t INA228_SetCurrentLSB(uint8_t dev_addr, uint32_t current_lsb) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL || current_lsb == 0) return -1;
    dev->current_lsb = current_lsb;
    return 0;
}

int32_t INA228_Write(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    return _INA228_Write(dev, reg_addr, (uint8_t *) &reg_data, INA228_16BREG);
}

int32_t INA228_Read16(uint8_t dev_addr, uint8_t reg_addr, uint16_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    return _INA228_Read(dev, reg_addr, (uint8_t *) reg_data, INA228_16BREG);
}

int32_t INA228_Read32(uint8_t dev_addr, uint8_t reg_addr, uint32_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    return _INA228_Read(dev, reg_addr, (uint8_t *) reg_data, INA228_24BREG);
}

int32_t INA228_Read64(uint8_t dev_addr, uint8_t reg_addr, uint64_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    return _INA228_Read(dev, reg_addr, (uint8_t *) reg_data, INA228_40BREG);
}

int32_t INA228_WriteCONFIG(uint8_t dev_addr, uint16_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    dev->adc_range = (*reg_data >> 4) & 1U;
    dev->voltage_res_den = dev->adc_range ? 8 : 2;    /* 78.125 nV/LSB : 312.5 nV/LSB */
    return _INA228_Write(dev, CONFIG, (uint8_t *) reg_data, INA228_16BREG);
}

/* takes SHUNT_CAL computed for ADCRANGE = 0 and applies the x4 scaling the
 * device requires when ADCRANGE = 1; the register is 15 bits wide, and CONFIG
 * must have been written first so that adc_range is current */
int32_t INA228_WriteSHUNT_CAL(uint8_t dev_addr, uint16_t shunt_cal) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    if (dev->adc_range) shunt_cal <<= 2;
    shunt_cal &= 0x7FFF;
    return _INA228_Write(dev, SHUNT_CAL, (uint8_t *) &shunt_cal, INA228_16BREG);
}

/* returns measured VSHUNT as nano-volts */
int32_t INA228_ReadVSHUNT(uint8_t dev_addr, int32_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    uint32_t raw = 0;
    if (dev == NULL) return -1;
    int32_t ret = _INA228_Read(dev, VSHUNT, (uint8_t *) &raw, INA228_24BREG);
    int32_t val = (int32_t)(raw << 8) >> 12;    /* sign-extend 20-bit two's complement at bits [23:4] */
    *reg_data = (int32_t)(((int64_t)val * voltage_res_num) / dev->voltage_res_den);
    return ret;
}

/* returns measured VBUS as micro-volts */
int32_t INA228_ReadVBUS(uint8_t dev_addr, uint32_t *reg_data) {
    int32_t ret = INA228_Read32(dev_addr, VBUS, reg_data);
    *reg_data >>= 4;
    *reg_data = (uint32_t)(((uint64_t)*reg_data * 3125) / 16);    /* 195.3125 = 3125/16 uV/LSB */
    return ret;
}

/* returns measured DIETEMP as millidegrees-C */
int32_t INA228_ReadDIETEMP(uint8_t dev_addr, int32_t *reg_data) {
    uint16_t raw;
    int32_t ret = INA228_Read16(dev_addr, DIETEMP, &raw);
    *reg_data = ((int32_t)(int16_t)raw * 125) / 16;    /* signed 16-bit reg; 7.8125 = 125/16 millidegrees-C/LSB */
    return ret;
}

/* returns measured CURRENT as nano-amps */
int32_t INA228_ReadCURRENT(uint8_t dev_addr, int32_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    uint32_t raw = 0;
    if (dev == NULL) return -1;
    int32_t ret = _INA228_Read(dev, CURRENT, (uint8_t *) &raw, INA228_24BREG);
    int32_t val = (int32_t)(raw << 8) >> 12;    /* sign-extend 20-bit two's complement at bits [23:4] */
    *reg_data = val * (int32_t)dev->current_lsb;
    return ret;
}

/* returns measured POWER as nano-Watts */
int32_t INA228_ReadPOWER(uint8_t dev_addr, uint64_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    uint32_t raw = 0;
    if (dev == NULL) return -1;
    int32_t ret = _INA228_Read(dev, POWER, (uint8_t *) &raw, INA228_24BREG);
    *reg_data = ((uint64_t)raw * dev->current_lsb * 16) / 5;    /* 3.2 = 16/5 */
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
    ina228_dev_t *dev = ina228_find(dev_addr);
    if (dev == NULL) return -1;
    int32_t ret = _INA228_Read(dev, ENERGY, (uint8_t *) reg_data, INA228_40BREG);
    *reg_data = (*reg_data * dev->current_lsb * 256) / 5;    /* 3.2 * 16 = 51.2 = 256/5 */
    return ret;
}

/* returns measured CHARGE as nano-Coulombs */
int32_t INA228_ReadCHARGE(uint8_t dev_addr, int64_t *reg_data) {
    ina228_dev_t *dev = ina228_find(dev_addr);
    uint64_t raw = 0;
    if (dev == NULL) return -1;
    int32_t ret = _INA228_Read(dev, CHARGE, (uint8_t *) &raw, INA228_40BREG);
    int64_t val = (int64_t)(raw << 24) >> 24;    /* sign-extend 40-bit two's complement */
    *reg_data = val * (int64_t)dev->current_lsb;
    return ret;
}
