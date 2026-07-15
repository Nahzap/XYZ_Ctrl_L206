/**
  ******************************************************************************
  * @file    app_rt.h
  * @brief   ADC XY sincronizado (TIM2) + filtros estables → C(z).
  *
  *   TIM6 @ 1 MHz     → átomo (app_atom_tick_1us)
  *   TIM2 @ 50 kHz    → TRGO dispara ADC1+ADC2 a la vez (X/Y sync)
  *   Sample 480 cicl. → Tconv≈18.2 µs @ ADCCLK 27 MHz (cabe en 20 µs)
  *   DMA HT/TC        → media recortada + EMA suave + C(z)
  ******************************************************************************
  */

#ifndef __APP_RT_H
#define __APP_RT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define APP_RT_ADC_HZ          50000UL
#define APP_RT_DMA_BLK         64U
#define APP_RT_CZ_PERIOD_US    640U

void app_rt_init(void);
void app_rt_tim6_isr(void);
void app_rt_dma_adc1_isr(void);

uint16_t app_rt_get_x(void);
uint16_t app_rt_get_y(void);
uint16_t app_rt_get_x_filtered(void);
uint16_t app_rt_get_y_filtered(void);
uint16_t app_rt_get_x_dwell(void);
uint16_t app_rt_get_y_dwell(void);
uint16_t app_rt_get_x_fast(void);
uint16_t app_rt_get_y_fast(void);
uint32_t app_rt_tick_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RT_H */
