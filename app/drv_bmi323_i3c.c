#include <stdio.h>

#include "board_defs.h"
#include "drv_bmi323.h"
#include "drv_i3c_bus.h"
#include "Driver_IO.h"
#include "pinconf.h"
#include "FreeRTOS.h"
#include "task.h"

/* An I2C read returns two dummy bytes before the register data. */
#define BMI323_READ_DUMMY_BYTES     2U
#define BMI323_MAX_READ_WORDS       3U

/* Feature engine start-up (datasheet "feature engine activation"). */
#define BMI323_FEATURE_IO2_STARTUP  0x012CU
#define BMI323_FEATURE_IO_SYNC      0x0001U
#define BMI323_FEATURE_ENGINE_EN    0x0001U
#define BMI323_FEATURE_STATUS_MASK  0x000FU     /* FEATURE_IO1.error_status */
#define BMI323_FEATURE_STATUS_OK    0x0001U     /* engine activated */
#define BMI323_FEATURE_START_MS     1000U

/* FEATURE_IO0: per-axis enables for no-motion [2:0] and any-motion [5:3]. */
#define BMI323_FEATURE_NO_MOTION_XYZ   0x0007U
#define BMI323_FEATURE_ANY_MOTION_XYZ  0x0038U

/* Extended (feature engine) registers, written through FEATURE_DATA_ADDR /
 * FEATURE_DATA_TX. Each detector is three words: threshold + acc_ref_up,
 * hysteresis, duration + wait time. */
#define BMI323_EXT_ANY_MOTION       0x05U
#define BMI323_EXT_NO_MOTION        0x08U
/* acc_ref_up = 1: the reference is updated every sample, so the detectors
 * compare the slope between consecutive samples (Bosch default). */
#define BMI323_MOTION_REF_UP        (1U << 12)

/* ACC_CONF: low-power mode [14:12] = 3, 2-sample average [10:8] = 1,
 * 8 g range [6:4] = 2, 50 Hz [3:0] = 7. The feature engine runs on 50 Hz data. */
#define BMI323_ACC_CONF_LP_50HZ     0x3127U

/* IO_INT_CTRL: INT1 active low [0] = 0, open-drain [1], output enabled [2].
 * The line idles high on the GPIO pad's internal pull-up (BMI323_IrqInit). */
#define BMI323_INT1_OPEN_DRAIN_LOW  0x0006U

/* INT_CONF: latched, cleared when INT_STATUS_INT1 is read. */
#define BMI323_INT_LATCHED          0x0001U
/* INT_MAP1: 2 bits per feature, 1 = INT1. no_motion [1:0], any_motion [3:2]. */
#define BMI323_MAP1_NO_MOTION_INT1  0x0001U
#define BMI323_MAP1_ANY_MOTION_INT1 0x0004U

/* ERR_REG: fatal_err [0], acc_conf_err [5]. */
#define BMI323_ERR_FATAL            0x0001U
#define BMI323_ERR_ACC_CONF         0x0020U

extern ARM_DRIVER_GPIO ARM_Driver_GPIO_(BOARD_BMI323_IMU_IRQ_GPIO_PORT);
static ARM_DRIVER_GPIO *GPIOdrv = &ARM_Driver_GPIO_(BOARD_BMI323_IMU_IRQ_GPIO_PORT);
/* GPIOn pin m has its own NVIC line: GPIO0_IRQ0 + 8 * n + m. */
#define BMI323_IRQ_IRQn ((IRQn_Type)(GPIO0_IRQ0_IRQn + 8 * BOARD_BMI323_IMU_IRQ_GPIO_PORT + \
                                     BOARD_BMI323_IMU_IRQ_GPIO_PIN))

static bmi323_irq_cb_t irq_cb;

int32_t BMI323_Write(uint8_t dev_addr, uint8_t reg_addr, uint16_t reg_data)
{
    uint8_t tx_data[3];

    tx_data[0] = reg_addr;
    tx_data[1] = (uint8_t)(reg_data & 0xFFU);   /* LSB first */
    tx_data[2] = (uint8_t)(reg_data >> 8);
    return i3c_bus_write(dev_addr, tx_data, sizeof(tx_data));
}

int32_t BMI323_Read(uint8_t dev_addr, uint8_t reg_addr, uint16_t *reg_data, uint8_t num_words)
{
    uint8_t rx_data[BMI323_READ_DUMMY_BYTES + 2U * BMI323_MAX_READ_WORDS];
    const uint8_t *p = &rx_data[BMI323_READ_DUMMY_BYTES];

    if (num_words == 0U || num_words > BMI323_MAX_READ_WORDS) {
        return -1;
    }
    if (i3c_bus_write_read(dev_addr, &reg_addr, 1U, rx_data,
                           (uint16_t)(BMI323_READ_DUMMY_BYTES + 2U * num_words)) != 0) {
        return -1;
    }
    for (uint8_t i = 0U; i < num_words; i++) {
        reg_data[i] = (uint16_t)(p[2U * i] | ((uint16_t)p[2U * i + 1U] << 8));
    }
    return 0;
}

/* Write consecutive extended registers starting at ext_addr. */
static int32_t BMI323_WriteExt(uint8_t dev_addr, uint8_t ext_addr,
                               const uint16_t *data, uint8_t num_words)
{
    uint8_t tx_data[1U + 2U * BMI323_MAX_READ_WORDS];

    if (num_words == 0U || num_words > BMI323_MAX_READ_WORDS) {
        return -1;
    }
    if (BMI323_Write(dev_addr, BMI323_FEATURE_DATA_ADDR, ext_addr) != 0) {
        return -1;
    }
    tx_data[0] = BMI323_FEATURE_DATA_TX;     /* the address auto-increments */
    for (uint8_t i = 0U; i < num_words; i++) {
        tx_data[1U + 2U * i] = (uint8_t)(data[i] & 0xFFU);
        tx_data[2U + 2U * i] = (uint8_t)(data[i] >> 8);
    }
    return i3c_bus_write(dev_addr, tx_data, (uint16_t)(1U + 2U * num_words));
}

int32_t BMI323_Init(uint8_t dev_addr)
{
    uint16_t value = 0U;
    uint32_t waited_ms = 0U;

    if (i3c_bus_init() != 0 || i3c_bus_attach_i2c(dev_addr) != 0) {
        return -1;
    }

    /* Validate that we are talking to the BMI323 */
    if (BMI323_Read(dev_addr, BMI323_CHIP_ID_REG, &value, 1U) != 0) {
        printf("BMI323: no response at 0x%02X\n", dev_addr);
        return -1;
    }
    if ((value & 0xFFU) != BMI323_CHIP_ID) {
        printf("BMI323: unexpected chip ID 0x%02X\n", (unsigned int)(value & 0xFFU));
        return -1;
    }

    /* Soft reset clears whatever a previous run (or a debugger restart) left
     * configured. The write itself may not be acknowledged cleanly. */
    (void)BMI323_Write(dev_addr, BMI323_CMD_REG, BMI323_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(10U));

    if (BMI323_Write(dev_addr, BMI323_FEATURE_IO2, BMI323_FEATURE_IO2_STARTUP) != 0 ||
        BMI323_Write(dev_addr, BMI323_FEATURE_IO_STATUS, BMI323_FEATURE_IO_SYNC) != 0 ||
        BMI323_Write(dev_addr, BMI323_FEATURE_CTRL, BMI323_FEATURE_ENGINE_EN) != 0) {
        return -1;
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10U));
        waited_ms += 10U;
        if (BMI323_Read(dev_addr, BMI323_FEATURE_IO1, &value, 1U) == 0 &&
            (value & BMI323_FEATURE_STATUS_MASK) == BMI323_FEATURE_STATUS_OK) {
            break;
        }
        if (waited_ms >= BMI323_FEATURE_START_MS) {
            printf("BMI323: feature engine did not start (FEATURE_IO1 0x%04X)\n",
                   (unsigned int)value);
            return -1;
        }
    }
    return 0;
}

int32_t BMI323_ConfigMotion(uint8_t dev_addr, const bmi323_motion_config_t *cfg)
{
    uint16_t any_motion[3];
    uint16_t no_motion[3];
    uint16_t err = 0U;

    any_motion[0] = (uint16_t)((cfg->any_threshold & 0x0FFFU) | BMI323_MOTION_REF_UP);
    any_motion[1] = (uint16_t)(cfg->any_hysteresis & 0x03FFU);
    any_motion[2] = (uint16_t)(cfg->any_duration & 0x1FFFU);   /* wait time 0 */
    no_motion[0] = (uint16_t)((cfg->no_threshold & 0x0FFFU) | BMI323_MOTION_REF_UP);
    no_motion[1] = (uint16_t)(cfg->no_hysteresis & 0x03FFU);
    no_motion[2] = (uint16_t)(cfg->no_duration & 0x1FFFU);

    if (BMI323_WriteExt(dev_addr, BMI323_EXT_ANY_MOTION, any_motion, 3U) != 0 ||
        BMI323_WriteExt(dev_addr, BMI323_EXT_NO_MOTION, no_motion, 3U) != 0) {
        return -1;
    }

    /* Enable both detectors on all axes; INT_MAP1 decides which one is heard. */
    if (BMI323_Write(dev_addr, BMI323_FEATURE_IO0,
                     BMI323_FEATURE_NO_MOTION_XYZ | BMI323_FEATURE_ANY_MOTION_XYZ) != 0 ||
        BMI323_Write(dev_addr, BMI323_FEATURE_IO_STATUS, BMI323_FEATURE_IO_SYNC) != 0) {
        return -1;
    }

    if (BMI323_Write(dev_addr, BMI323_ACC_CONF, BMI323_ACC_CONF_LP_50HZ) != 0 ||
        BMI323_Read(dev_addr, BMI323_ERROR_REG, &err, 1U) != 0) {
        return -1;
    }
    if ((err & (BMI323_ERR_FATAL | BMI323_ERR_ACC_CONF)) != 0U) {
        printf("BMI323: ACC_CONF rejected (ERR_REG 0x%04X)\n", (unsigned int)err);
        return -1;
    }

    if (BMI323_Write(dev_addr, BMI323_IO_INT_CTRL, BMI323_INT1_OPEN_DRAIN_LOW) != 0 ||
        BMI323_Write(dev_addr, BMI323_INT_CONF, BMI323_INT_LATCHED) != 0 ||
        BMI323_Write(dev_addr, BMI323_INT_MAP1, 0U) != 0) {
        return -1;
    }
    return 0;
}

int32_t BMI323_MapInt1(uint8_t dev_addr, uint16_t int_mask)
{
    uint16_t map = 0U;

    if ((int_mask & BMI323_INT_NO_MOTION) != 0U) {
        map |= BMI323_MAP1_NO_MOTION_INT1;
    }
    if ((int_mask & BMI323_INT_ANY_MOTION) != 0U) {
        map |= BMI323_MAP1_ANY_MOTION_INT1;
    }
    return BMI323_Write(dev_addr, BMI323_INT_MAP1, map);
}

int32_t BMI323_ReadIntStatus1(uint8_t dev_addr, uint16_t *status)
{
    return BMI323_Read(dev_addr, BMI323_INT_STATUS_INT1, status, 1U);
}

int32_t BMI323_ReadAcc(uint8_t dev_addr, xyz_accel_s *acc)
{
    uint16_t raw[3];

    if (BMI323_Read(dev_addr, BMI323_ACC_X_REG, raw, 3U) != 0) {
        return -1;
    }
    acc->x = (int16_t)raw[0];
    acc->y = (int16_t)raw[1];
    acc->z = (int16_t)raw[2];
    return 0;
}

static void BMI323_GPIO_CB(uint32_t event)
{
    (void)event;
    if (irq_cb != NULL) {
        irq_cb();
    }
}

int32_t BMI323_IrqInit(bmi323_irq_cb_t cb)
{
    /* INT1 is open-drain, active low: interrupt on the falling edge. */
    uint32_t arg = ARM_GPIO_IRQ_POLARITY_LOW | ARM_GPIO_IRQ_EDGE_SENSITIVE_SINGLE |
                   ARM_GPIO_IRQ_SENSITIVE_EDGE;
    const uint8_t pin = BOARD_BMI323_IMU_IRQ_GPIO_PIN;

    irq_cb = cb;
    /* GPIO function with the input buffer and the internal pull-up enabled,
     * set here so it doesn't depend on the board's pins.h. */
    if (pinconf_set(BOARD_BMI323_IMU_IRQ_GPIO_PORT, pin, PINMUX_ALTERNATE_FUNCTION_0,
                    PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP) != 0 ||
        GPIOdrv->Initialize(pin, BMI323_GPIO_CB) != ARM_DRIVER_OK ||
        GPIOdrv->PowerControl(pin, ARM_POWER_FULL) != ARM_DRIVER_OK ||
        GPIOdrv->SetDirection(pin, GPIO_PIN_DIRECTION_INPUT) != ARM_DRIVER_OK ||
        GPIOdrv->Control(pin, ARM_GPIO_ENABLE_INTERRUPT, &arg) != ARM_DRIVER_OK) {
        printf("BMI323: IRQ GPIO%u_%u setup failed\n",
               (unsigned int)BOARD_BMI323_IMU_IRQ_GPIO_PORT, (unsigned int)pin);
        return -1;
    }
    /* The driver applies RTE_GPIOn_PINm_IRQ_PRIORITY (0), above the FreeRTOS
     * syscall ceiling; the callback gives a semaphore, so lower it. */
    NVIC_SetPriority(BMI323_IRQ_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    return 0;
}
