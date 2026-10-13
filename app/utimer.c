#include "board_defs.h"
#include "pinconf.h"
#include "pm_soc_clk.h"
#include "sys_clocks.h"
#include "utimer.h"
#include "utimer_pwm.h"

/* LCD backlight PWM on UT4_T1_B (P6_1, alternate function 4): UTIMER channel 4,
 * driver B. Only channel 4's bits in the global registers are touched. */
#define UTIMER_CHANNEL  4U
#define UTIMER_PINMUX   PINMUX_ALTERNATE_FUNCTION_4
#define UTIMER_PERIOD   651     /* microseconds */

#define UT_COMPARE_CFG0 0x919   /* 1 at start of cycle, 0 on compare match */
#define UT_COMPARE_CFG1 0x906   /* 0 at start of cycle, 1 on compare match */

/* Counter top (CNTR_PTR); the counter runs 0..top, so a period is top + 1. */
static uint32_t ut_top;

static void configure_utimer_pinmux(void)
{
    uint32_t padconf = PADCTRL_OUTPUT_DRIVE_STRENGTH_8MA;
    pinconf_set(BOARD_LCD_BACKLIGHT_GPIO_PORT, BOARD_LCD_BACKLIGHT_GPIO_PIN,
                UTIMER_PINMUX, padconf);
}

static void configure_utimer(uint16_t duty_permille)
{
    volatile UTIMER_UTIMER_CHANNEL_CFG_Type *UT4 = &UTIMER->UTIMER_CHANNEL_CFG[UTIMER_CHANNEL];
    const uint32_t ch_bit = 1UL << UTIMER_CHANNEL;

    uint64_t ut_cnt_max = SystemAXIClock;
    ut_cnt_max *= UTIMER_PERIOD;
    ut_cnt_max /= 1000000;
    ut_top = (uint32_t)ut_cnt_max - 1U;

    UTIMER->UTIMER_GLB_CNTR_STOP |= ch_bit;         /* stop the UT4 counter */
    UTIMER->UTIMER_GLB_CNTR_CLEAR |= ch_bit;        /* clear its count value */
    UTIMER->UTIMER_GLB_CLOCK_ENABLE |= ch_bit;      /* enable the UT4 clock */
    UTIMER->UTIMER_GLB_DRIVER_OEN &= ~(GLB_DRIVER_CHAN_B_OEN << (UTIMER_CHANNEL << 1));  /* enable output B */

    UT4->UTIMER_START_1_SRC = CNTR_SRC1_PGM_EN;     /* enable programmatic start */
    UT4->UTIMER_CNTR_CTRL = CNTR_CTRL_EN;           /* enable the counter (sawtooth, up) */
    UT4->UTIMER_CNTR = 0;                           /* clear the count value */
    UT4->UTIMER_CNTR_PTR = ut_top;                  /* set the counter overflow value */
    /* output is 1 at start of cycle, 0 on compare match: duty = compare / period */
    UT4->UTIMER_COMPARE_CTRL_B = UT_COMPARE_CFG0;
    utimer_pwm_set_duty(duty_permille);

    UTIMER->UTIMER_GLB_CNTR_START |= ch_bit;        /* start the UT4 counter */
}

void utimer_pwm_init(uint16_t duty_permille)
{
    pm_soc_clk_get_busclk();
    configure_utimer(duty_permille);
    configure_utimer_pinmux();      /* switch the pin over once the PWM runs */
}

/* COMPARE_B can change mid-cycle; at worst one 651 us period comes out at
 * 0 % or 100 %, which is invisible on a backlight. */
void utimer_pwm_set_duty(uint16_t duty_permille)
{
    volatile UTIMER_UTIMER_CHANNEL_CFG_Type *UT4 = &UTIMER->UTIMER_CHANNEL_CFG[UTIMER_CHANNEL];

    if (duty_permille == 0U) {
        /* A disabled driver outputs its disable value, 0 in UT_COMPARE_CFG0. */
        UT4->UTIMER_COMPARE_CTRL_B &= ~COMPARE_CTRL_DRV_DRIVER_EN;
        return;
    }

    if (duty_permille >= UTIMER_PWM_DUTY_MAX) {
        UT4->UTIMER_COMPARE_B = ut_top + 1U;        /* never matches: stays 1 */
    } else {
        uint64_t compare = (uint64_t)(ut_top + 1U) * duty_permille;
        UT4->UTIMER_COMPARE_B = (uint32_t)(compare / UTIMER_PWM_DUTY_MAX);
    }
    UT4->UTIMER_COMPARE_CTRL_B |= COMPARE_CTRL_DRV_DRIVER_EN;
}
