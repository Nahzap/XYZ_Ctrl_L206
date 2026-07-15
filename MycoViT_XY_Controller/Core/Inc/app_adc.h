/**
  ******************************************************************************
  * @file    app_adc.h
  * @brief   Application ADC reads (polling, software trigger, 12-bit).
  *          POT_A/POT_B via ADC3 scan (PF5/PC0). Laser Y/X via app_rt @ 1 MHz
 *          (PC3/ADC1 IN13, PA3/ADC2 IN3) — not read from this module.
  ******************************************************************************
  */

#ifndef __APP_ADC_H
#define __APP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Enable ADC3 (pots) for polled reads. ADC1/ADC2 (laser X/Y) are owned by the
   1 MHz real-time loop (see app_rt.c). */
void app_adc_init(void);

/* Read both potentiometers in a single ADC3 scan (0..4095 each). */
void app_adc_read_pots(uint16_t *pot_a, uint16_t *pot_b);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ADC_H */
