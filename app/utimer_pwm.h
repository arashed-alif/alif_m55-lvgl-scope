#ifndef UTIMER_PWM_H
#define UTIMER_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UTIMER_PWM_DUTY_MAX 1000U   /* duty is in permille */

/* Take over the LCD backlight pin (P6_1) as UT4_T1_B and start a 1536 Hz PWM
 * at the given duty. Call after clock_init(), which changes the AXI clock. */
void utimer_pwm_init(uint16_t duty_permille);

/* Change the duty; 0 holds the pin low and UTIMER_PWM_DUTY_MAX holds it high. */
void utimer_pwm_set_duty(uint16_t duty_permille);

#ifdef __cplusplus
}
#endif

#endif
